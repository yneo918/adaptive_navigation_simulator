"""Batch experiment orchestrator.

Runs a list of AN experiments back-to-back against a persistent simulator
stack. For each entry it:
  1. Spawns trajectory_plotter_3d as a subprocess (saves NPZ on SIGINT).
  2. Publishes /auto/config (d, adaptive_mode) and waits for warmup.
  3. Publishes /rviz/pose2D for initial cluster placement.
  4. Publishes /auto/start to enable navigation.
  5. Subscribes /p1/pose2D, monitors termination conditions.
  6. Publishes /auto/stop, then SIGINTs the plotter so it writes the NPZ.
  7. Moves the timestamped NPZ into the experiment's output_subdir.

Prerequisite: launch disaster_AN_headless.launch.py in another shell first.

Usage:
    ros2 run auto_runner batch_orchestrator \\
        --config /path/to/experiments.yaml \\
        --output-root /path/to/data \\
        [--ids B1_MAX_d10,D1_CT_d10]
"""

import argparse
import collections
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String, Float64

# RCLError is the catch-all for rcl-layer failures (e.g., publishing on a node
# whose context was shut down). It lives under different paths across rclpy
# versions — try the stable one, fall back to the pybind11 module.
try:
    from rclpy.exceptions import RCLError  # type: ignore[attr-defined]
except ImportError:
    from rclpy import _rclpy_pybind11 as _rclpy_impl  # noqa: F401
    RCLError = _rclpy_impl.RCLError


PLOTTER_OUTPUT_DIR = '/tmp/auto_runner_plotter'


class Orchestrator(Node):
    """Single-shot orchestrator node owning publishers, pose subscriber."""

    def __init__(self):
        super().__init__('batch_orchestrator')
        # Tracks the plotter of the currently running experiment so abort paths
        # can reap it; set in run_one_experiment, cleared after each clean stop.
        self._active_plotter = None
        self.config_pub = self.create_publisher(String, '/auto/config', 10)
        self.start_pub = self.create_publisher(Bool, '/auto/start', 10)
        self.stop_pub = self.create_publisher(Bool, '/auto/stop', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/rviz/pose2D', 10)
        # Stuck-escape: ask the AN node to rotate about the centroid (target
        # angle [rad]); _escape_active tracks whether that rotation is running.
        self.escape_pub = self.create_publisher(Float64, '/ctrl/an_escape', 10)
        self._escape_active = False
        self.create_subscription(
            Bool, '/ctrl/an_escape_active', self._on_escape_active, 10)
        # Simulation clock (published by clock_publisher, advances at
        # time_scale x wall). Sampling/step counting must follow it so one
        # step is always sample_interval SIMULATED seconds regardless of the
        # acceleration; wall time is kept only for safety caps and warmup.
        self._sim_time_s = None
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

        self._leader_xy = None
        self._leader_history = collections.deque(maxlen=4000)
        self._z_c_history = collections.deque(maxlen=4000)
        self.create_subscription(
            Pose2D, '/p1/pose2D', self._on_leader_pose, 10)

        # Out-of-area detection: track latest sensor reading per robot
        self._robot_z = {f'p{i}': None for i in range(1, 6)}
        self._oob_streak = 0
        for i in range(1, 6):
            rid = f'p{i}'
            self.create_subscription(
                Float64, f'/{rid}/sensor',
                lambda msg, r=rid: self._on_robot_sensor(msg, r),
                10)

    def _on_leader_pose(self, msg: Pose2D):
        self._leader_xy = (msg.x, msg.y)

    def _on_robot_sensor(self, msg: Float64, robot_id: str):
        self._robot_z[robot_id] = float(msg.data)

    def _on_escape_active(self, msg: Bool):
        self._escape_active = bool(msg.data)

    def _on_clock(self, msg: Clock):
        self._sim_time_s = msg.clock.sec + msg.clock.nanosec / 1e9

    def now_s(self) -> float:
        """Sim time [s] if /clock has been seen, else wall time."""
        return self._sim_time_s if self._sim_time_s is not None \
            else time.time()

    def wait_for_leader(self, timeout_s: float = 30.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline and self._leader_xy is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._leader_xy is not None

    def reset_history(self):
        self._leader_history.clear()
        self._z_c_history.clear()
        self._leader_xy = None
        self._oob_streak = 0
        for k in self._robot_z:
            self._robot_z[k] = None

    def push_history(self):
        if self._leader_xy is not None:
            self._leader_history.append(self._leader_xy)
        # Track mean sensor reading across all reporting robots as z_c proxy.
        # Used as one of the convergence signals (orbital limit-cycle detection).
        z_vals = [v for v in self._robot_z.values() if v is not None]
        if z_vals:
            self._z_c_history.append(sum(z_vals) / len(z_vals))

    def push_oob_check(self, threshold: float) -> int:
        """Update out-of-area streak counter. Returns current streak.

        Out-of-area: ALL 5 robots have sensor z < threshold simultaneously.
        Streak increments per call when condition holds; resets otherwise.
        """
        if any(z is None for z in self._robot_z.values()):
            return self._oob_streak  # not all robots reporting yet
        if max(self._robot_z.values()) < threshold:
            self._oob_streak += 1
        else:
            self._oob_streak = 0
        return self._oob_streak


def import_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def make_pose_msg(start: dict) -> Pose2D:
    msg = Pose2D()
    msg.x = float(start['x'])
    msg.y = float(start['y'])
    msg.theta = float(start.get('theta', 0.0))
    return msg


def spawn_plotter(plotter_dir: str, sample_interval: float,
                  log_path: str | None = None) -> subprocess.Popen:
    """Spawn trajectory_plotter_3d in its own process group so SIGINT is isolated.

    If log_path is provided, both stdout and stderr are written there so a
    failed NPZ save can be diagnosed post-mortem.
    """
    os.makedirs(plotter_dir, exist_ok=True)
    args = [
        'ros2', 'run', 'sensor_field', 'trajectory_plotter_3d',
        '--ros-args',
        '-p', f'output_dir:={plotter_dir}',
        '-p', f'trajectory_sample_interval:={sample_interval}',
        # Sample on the scaled /clock so NPZ rows are sample_interval
        # SIMULATED seconds apart at any time_scale.
        '-p', 'use_sim_time:=true',
        '-p', 'visualization_mode:=contour',  # avoid blocking 3D show
        '-p', 'show_plot:=false',  # no GUI popup in batch mode
        # cesium_field.yaml uses distance_scale=0.05 (1 sim unit = 20 m real)
        # so display_scale_xy = 1/0.05 = 20 -> axis values in real meters.
        '-p', 'sensor_distance_scale:=0.05',
        '-p', 'original_coordinate_unit:=m',
    ]
    if log_path:
        log_f = open(log_path, 'w')
        return subprocess.Popen(
            args,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    return subprocess.Popen(
        args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop_plotter(proc: subprocess.Popen, save_timeout_s: float = 60.0,
                 verbose: bool = False) -> str:
    """Send SIGINT to plotter process group; wait for clean exit (NPZ save).

    Returns a short status string explaining how the plotter exited.
    """
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return 'already-dead'
    try:
        rc = proc.wait(timeout=save_timeout_s)
        if rc == 0:
            return f'clean-exit(rc={rc})'
        return f'failed-exit(rc={rc})'
    except subprocess.TimeoutExpired:
        if verbose:
            print(f'[stop_plotter] SIGINT timeout after {save_timeout_s}s, '
                  f'escalating to SIGTERM', file=sys.stderr)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            return 'sigint-timeout-then-dead'
        try:
            rc = proc.wait(timeout=5.0)
            return f'sigterm-exit(rc={rc})'
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            return 'sigkill'


def kill_plotter_group(proc) -> None:
    """Best-effort hard kill of a plotter and its whole process group.

    Used on abort/error paths (Ctrl+C -> RCLError, unexpected exceptions,
    KeyboardInterrupt). Unlike stop_plotter(), this does NOT send SIGINT and
    does NOT wait for an NPZ save: an aborted run never reaches the step-8 NPZ
    move, so a partial save would only linger in the plotter output dir and be
    wiped by the next run. The point here is purely to prevent orphans, since
    the plotter runs in its own session (os.setsid) and is therefore immune to
    the terminal's Ctrl+C. A no-op if proc is None or already exited.
    """
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, OSError):
        pass


def already_completed(output_root: str, expt: dict) -> str | None:
    """Return path of an existing completed NPZ for this experiment, if any.

    Matches files of the form `<expt_id>_*_trajectory_data.npz` placed by a
    previous run in the experiment's output_subdir.
    """
    out_dir = os.path.join(output_root, expt['output_subdir'])
    if not os.path.isdir(out_dir):
        return None
    eid = expt['id']
    for name in os.listdir(out_dir):
        if name.startswith(f'{eid}_') and name.endswith('_trajectory_data.npz'):
            return os.path.join(out_dir, name)
    return None


def find_latest_npz(plotter_dir: str, since_ts: float) -> str | None:
    """Return path to NPZ created after `since_ts` (max one expected)."""
    candidates = []
    for name in os.listdir(plotter_dir):
        if not name.endswith('_trajectory_data.npz'):
            continue
        full = os.path.join(plotter_dir, name)
        if os.path.getmtime(full) >= since_ts - 1.0:
            candidates.append((os.path.getmtime(full), full))
    if not candidates:
        return None
    candidates.sort()
    return candidates[-1][1]


def check_termination(orch: Orchestrator, expt: dict, defaults: dict,
                      step_count: int, t_started: float) -> tuple[bool, str]:
    """Return (terminate, reason) based on max_steps / time / plateau."""
    max_steps = expt.get('max_steps', defaults['max_steps'])
    if step_count >= max_steps:
        return True, f'max_steps={max_steps}'

    time_limit = expt.get('time_limit_s', defaults['time_limit_s'])
    if time.time() - t_started >= time_limit:
        return True, f'time_limit={time_limit}s'

    # RANDOM_WALK has no "convergence" by design (it wanders), so plateau /
    # net_disp / z_range checks misfire: a wandering trajectory has near-zero
    # net displacement over any window. Always disable convergence checks for
    # RANDOM_WALK regardless of YAML setting.
    mode_for_term = expt.get('mode', '')
    rw_mode = mode_for_term in ('RANDOM_WALK', 'random_walk')

    if (not rw_mode) and (not expt.get('disable_plateau',
                                       defaults.get('disable_plateau', False))):
        win = expt.get('plateau_window_steps', defaults['plateau_window_steps'])
        eps = expt.get('plateau_eps', defaults['plateau_eps'])
        net_eps = expt.get('net_disp_eps', defaults.get('net_disp_eps', 2.0))
        z_eps = expt.get('z_range_eps', defaults.get('z_range_eps', 0.005))

        if len(orch._leader_history) >= win:
            recent = list(orch._leader_history)[-win:]
            xs = [p[0] for p in recent]
            ys = [p[1] for p in recent]

            # (1) Position span: detects true stop (rare with diff-drive)
            span = max(max(xs) - min(xs), max(ys) - min(ys))
            if span < eps:
                return True, f'plateau (span={span:.3f} < {eps})'

            # (2) Net displacement: catches orbital limit cycles around an
            # extremum even when oscillation amplitude exceeds plateau_eps.
            dx = recent[-1][0] - recent[0][0]
            dy = recent[-1][1] - recent[0][1]
            net = (dx * dx + dy * dy) ** 0.5
            if net < net_eps:
                return True, (f'net_disp converged '
                              f'({net:.3f} < {net_eps} over {win} steps)')

        # (3) z_c stability AND recent stillness
        # Bare z_c stability can fire while the cluster is still actively
        # moving along an iso-deposition contour (perpendicular to gradient).
        # That is NOT a convergence in the MAX/MIN sense (the cluster could
        # in principle escape if pointed correctly). Require RECENT position
        # span (last ~200 steps) to also be small. Path length is a poor
        # discriminator because a cluster oscillating in place still
        # accumulates path; what we want is the spatial extent of recent
        # motion — small means "parked".
        if len(orch._z_c_history) >= win:
            recent_z = list(orch._z_c_history)[-win:]
            z_range = max(recent_z) - min(recent_z)
            if z_range < z_eps:
                motion_win = expt.get(
                    'stable_motion_window_steps',
                    defaults.get('stable_motion_window_steps', 200))
                motion_span_eps = expt.get(
                    'stable_motion_span_eps',
                    defaults.get('stable_motion_span_eps', 5.0))
                if len(orch._leader_history) >= motion_win:
                    pts = list(orch._leader_history)[-motion_win:]
                    xs2 = [p[0] for p in pts]
                    ys2 = [p[1] for p in pts]
                    span2 = max(max(xs2) - min(xs2), max(ys2) - min(ys2))
                    if span2 < motion_span_eps:
                        return True, (
                            f'z_c stable + parked '
                            f'(z_range={z_range:.4f}<{z_eps}, '
                            f'span={span2:.2f}<{motion_span_eps} '
                            f'in last {motion_win} steps)')
                    # else: iso-contour traversal, do NOT terminate

    # Out-of-area: all 5 robots below sensor threshold for sustained period
    oob_win = expt.get('oob_window_steps', defaults['oob_window_steps'])
    if orch._oob_streak >= oob_win:
        thr = expt.get('oob_threshold', defaults['oob_threshold'])
        return True, (f'out-of-area (all robots z<{thr} '
                      f'for {orch._oob_streak} steps)')

    return False, ''


# Termination reasons that represent a (possibly false) CONVERGENCE, as opposed
# to hard caps (max_steps/time_limit) or leaving the data region (out-of-area).
# Only these are eligible for the stuck-escape rotation.
_CONVERGENCE_PREFIXES = ('plateau', 'net_disp', 'z_c stable')


def _is_convergence(reason: str) -> bool:
    return reason.startswith(_CONVERGENCE_PREFIXES)


def do_escape(orch: 'Orchestrator', angle: float,
              timeout_s: float = 120.0, act_timeout_s: float = 5.0) -> bool:
    """Command a centroid-rotation escape and block until the AN node finishes.

    Publishes the target angle [rad] on /ctrl/an_escape, then spins until the
    node reports it started (an_escape_active True) and then completed (False).
    Always clears the command afterwards so a stale value cannot re-trigger.
    Returns True if the rotation actually ran.
    """
    for _ in range(3):  # a few publishes in case of drops
        orch.escape_pub.publish(Float64(data=float(angle)))
        rclpy.spin_once(orch, timeout_sec=0.02)
    seen_active = False
    start = time.time()
    while time.time() - start < timeout_s:
        rclpy.spin_once(orch, timeout_sec=0.05)
        if orch._escape_active:
            seen_active = True
        elif seen_active:
            break  # was active, now finished
        elif time.time() - start > act_timeout_s:
            break  # never acknowledged (e.g. node disabled) -> give up
    orch.escape_pub.publish(Float64(data=0.0))
    rclpy.spin_once(orch, timeout_sec=0.05)
    return seen_active


def run_one_experiment(orch: Orchestrator, expt: dict, defaults: dict,
                       output_root: str) -> bool:
    """Execute a single experiment. Return True on success."""
    eid = expt['id']
    mode = expt['mode']
    d = float(expt['d'])
    sample_interval = expt.get(
        'trajectory_sample_interval', defaults['trajectory_sample_interval'])
    warmup = expt.get('warmup_s', defaults['warmup_s'])
    cooldown = expt.get('cooldown_s', defaults['cooldown_s'])

    print(f'\n{"=" * 60}\n[{eid}] mode={mode} d={d}\n{"=" * 60}')

    # Reset orchestrator state
    orch.reset_history()

    # 1. Configure headless controller
    cfg = {'d': d, 'adaptive_mode': mode}
    if 'z_des' in expt:
        cfg['z_des'] = float(expt['z_des'])
    orch.config_pub.publish(String(data=__import__('json').dumps(cfg)))
    rclpy.spin_once(orch, timeout_sec=0.1)
    # Publish twice to ensure the late subscribers receive it
    for _ in range(3):
        rclpy.spin_once(orch, timeout_sec=0.1)
    orch.config_pub.publish(String(data=__import__('json').dumps(cfg)))

    # 1b. For RANDOM_WALK baseline, push seed/period directly to adaptive_nav
    # via ros2 param set (the parameter callback re-seeds the RNG immediately).
    # The adaptive_nav node is launched as /cluster_feedback (per launch file).
    if mode in ('RANDOM_WALK', 'random_walk'):
        seed = int(expt.get('random_walk_seed', 0))
        period = int(expt.get(
            'random_walk_period_steps',
            defaults.get('random_walk_period_steps', 50)))
        # Try common node names; the disaster_AN_headless.launch.py uses
        # `name='cluster_feedback'` for adaptive_nav. Fall back to
        # `adaptive_navigator` in case the launch is changed.
        for node_name in ('/cluster_feedback', '/adaptive_navigator'):
            success = True
            for param, val in [('random_walk_seed', seed),
                               ('random_walk_period_steps', period)]:
                try:
                    subprocess.run(
                        ['ros2', 'param', 'set', node_name, param, str(val)],
                        check=True, capture_output=True, timeout=10.0)
                except (subprocess.CalledProcessError,
                        subprocess.TimeoutExpired) as e:
                    success = False
                    break
            if success:
                print(f'[{eid}] random_walk: seed={seed}, period={period} '
                      f'(applied to {node_name})')
                break
        else:
            print(f'[{eid}] WARN: ros2 param set failed on all known node names; '
                  f'random_walk_seed will be node default',
                  file=sys.stderr)

    # 2. Spawn the trajectory plotter, capturing its log for diagnostics
    shutil.rmtree(PLOTTER_OUTPUT_DIR, ignore_errors=True)
    os.makedirs(PLOTTER_OUTPUT_DIR, exist_ok=True)
    t_spawn = time.time()
    plotter_log = os.path.join(PLOTTER_OUTPUT_DIR, f'{eid}_plotter.log')
    plotter = spawn_plotter(PLOTTER_OUTPUT_DIR, sample_interval,
                            log_path=plotter_log)
    # Record the in-flight plotter so abort/error paths in main() can reap it;
    # cleared again after every clean stop_plotter() below.
    orch._active_plotter = plotter
    print(f'[{eid}] plotter spawned (pid={plotter.pid}, log={plotter_log})')

    # 3. Wait for plotter terrain ingestion + first leader pose
    print(f'[{eid}] warmup {warmup}s ...')
    deadline = time.time() + warmup
    while time.time() < deadline:
        rclpy.spin_once(orch, timeout_sec=0.1)

    # 4. Publish initial pose
    orch.pose_pub.publish(make_pose_msg(expt['start']))
    print(f'[{eid}] initial pose published')
    # Re-publish a few times to ensure delivery
    for _ in range(5):
        rclpy.spin_once(orch, timeout_sec=0.1)
    orch.pose_pub.publish(make_pose_msg(expt['start']))

    if not orch.wait_for_leader(timeout_s=15.0):
        print(f'[{eid}] ERROR: leader pose never received', file=sys.stderr)
        stop_plotter(plotter)
        orch._active_plotter = None
        return False

    # Settle for ~2s after pose set so robots are stationary at start
    settle_until = time.time() + 2.0
    while time.time() < settle_until:
        rclpy.spin_once(orch, timeout_sec=0.1)

    # 5. Start navigation
    orch.start_pub.publish(Bool(data=True))
    t_started = time.time()
    print(f'[{eid}] /auto/start sent')

    # 6. Main loop: monitor termination
    step_count = 0
    last_sample_t = orch.now_s()
    print(f'[{eid}] sampling on '
          f'{"sim clock" if orch._sim_time_s is not None else "WALL clock (no /clock seen)"}')
    progress_every = expt.get('progress_every_steps',
                              defaults.get('progress_every_steps', 20))

    # Stuck-escape (opt-in). On a CONVERGENCE detection, rotate about the
    # centroid instead of terminating, up to escape_cap times per spatial area
    # (radius escape_R ~ cluster size). Truly-converged runs hit the cap and
    # then terminate as before.
    enable_escape = expt.get('enable_escape',
                             defaults.get('enable_escape', False))
    escape_angle = float(expt.get('escape_angle',
                                  defaults.get('escape_angle', math.pi / 2)))
    escape_cap = int(expt.get('escape_cap', defaults.get('escape_cap', 2)))
    escape_R = float(expt.get('escape_area_factor',
                              defaults.get('escape_area_factor', 2.0))) * d
    escape_areas = []  # list of [cx, cy, count]

    while True:
        rclpy.spin_once(orch, timeout_sec=0.05)
        # Sample at the same interval as the plotter so step_count tracks NPZ rows
        if orch.now_s() - last_sample_t >= sample_interval:
            orch.push_history()
            oob_thr = expt.get('oob_threshold', defaults['oob_threshold'])
            orch.push_oob_check(oob_thr)
            step_count += 1
            last_sample_t = orch.now_s()

            # Periodic progress line so long runs are not silent
            if progress_every > 0 and step_count % progress_every == 0:
                lx, ly = orch._leader_xy if orch._leader_xy else (0.0, 0.0)
                z_vals = [v for v in orch._robot_z.values() if v is not None]
                z_c = (sum(z_vals) / len(z_vals)) if z_vals else float('nan')
                z_max = max(z_vals) if z_vals else float('nan')
                # Leader displacement over plateau window for convergence sense
                hist = list(orch._leader_history)[-defaults['plateau_window_steps']:]
                if len(hist) >= 2:
                    xs = [p[0] for p in hist]
                    ys = [p[1] for p in hist]
                    span = max(max(xs) - min(xs), max(ys) - min(ys))
                else:
                    span = float('nan')
                print(f'[{eid}] step={step_count:>4} '
                      f'leader=({lx:>7.1f},{ly:>7.1f}) '
                      f'z_c={z_c:.3f} z_max={z_max:.3f} '
                      f'span={span:.2f} oob_streak={orch._oob_streak}')

        terminate, reason = check_termination(
            orch, expt, defaults, step_count, t_started)
        if terminate:
            if enable_escape and _is_convergence(reason):
                lx, ly = orch._leader_xy if orch._leader_xy else (0.0, 0.0)
                area = next(
                    (a for a in escape_areas
                     if math.hypot(lx - a[0], ly - a[1]) <= escape_R), None)
                if area is None:
                    area = [lx, ly, 0]
                    escape_areas.append(area)
                if area[2] < escape_cap:
                    area[2] += 1
                    print(f'[{eid}] convergence ({reason}); escape rotate '
                          f'#{area[2]}/{escape_cap} at ({lx:.1f},{ly:.1f}), '
                          f'step={step_count}')
                    do_escape(orch, escape_angle)
                    # Re-arm convergence detectors after the maneuver.
                    orch._leader_history.clear()
                    orch._z_c_history.clear()
                    orch._oob_streak = 0
                    last_sample_t = orch.now_s()
                    continue
                print(f'[{eid}] convergence ({reason}); escape cap reached '
                      f'at ({lx:.1f},{ly:.1f}) -> terminating '
                      f'(step={step_count})')
                break
            print(f'[{eid}] terminating: {reason} (step={step_count})')
            break

    # 7. Stop navigation, cool down, signal plotter
    orch.stop_pub.publish(Bool(data=True))
    cool_deadline = time.time() + cooldown
    while time.time() < cool_deadline:
        rclpy.spin_once(orch, timeout_sec=0.05)
    print(f'[{eid}] SIGINT plotter ...')
    status = stop_plotter(plotter, verbose=True)
    orch._active_plotter = None
    print(f'[{eid}] plotter {status}')

    # Brief grace for filesystem flush after the plotter exits
    time.sleep(1.0)

    # 8. Move NPZ to organized location with experiment id prefix
    npz_path = find_latest_npz(PLOTTER_OUTPUT_DIR, t_spawn)
    if npz_path is None:
        print(f'[{eid}] ERROR: no NPZ produced (plotter status: {status})',
              file=sys.stderr)
        # Preserve the plotter log so the failure can be diagnosed
        if os.path.isfile(plotter_log):
            out_dir = os.path.join(output_root, expt['output_subdir'])
            os.makedirs(out_dir, exist_ok=True)
            saved_log = os.path.join(out_dir, f'{eid}_FAILED_plotter.log')
            try:
                shutil.copy(plotter_log, saved_log)
                print(f'[{eid}] plotter log preserved -> {saved_log}',
                      file=sys.stderr)
                # Dump the tail so the user sees the cause immediately
                with open(plotter_log) as f:
                    lines = f.readlines()
                tail = ''.join(lines[-15:])
                print(f'[{eid}] last 15 lines of plotter log:\n{tail}',
                      file=sys.stderr)
            except OSError as e:
                print(f'[{eid}] could not preserve plotter log: {e}',
                      file=sys.stderr)
        return False

    out_dir = os.path.join(output_root, expt['output_subdir'])
    os.makedirs(out_dir, exist_ok=True)
    # Prefix the experiment id so resume / skip-existing can match it later
    base = os.path.basename(npz_path)
    dest = os.path.join(out_dir, f'{eid}_{base}')
    shutil.move(npz_path, dest)
    print(f'[{eid}] saved -> {dest}')

    # Also move any companion PNG/PDF artifacts the plotter saved alongside
    # the NPZ (same timestamp prefix, e.g. 20260507_120100_trajectory_3d_contour.png)
    # so all per-run outputs land in one directory.
    npz_dir = os.path.dirname(npz_path)
    timestamp_prefix = base.split('_trajectory_')[0]  # YYYYMMDD_HHMMSS
    if os.path.isdir(npz_dir):
        for name in os.listdir(npz_dir):
            if name.startswith(timestamp_prefix) and not name.endswith(
                    '_trajectory_data.npz'):
                src = os.path.join(npz_dir, name)
                aux_dest = os.path.join(out_dir, f'{eid}_{name}')
                try:
                    shutil.move(src, aux_dest)
                    print(f'[{eid}] aux  -> {aux_dest}')
                except OSError as e:
                    print(f'[{eid}] warning: failed to move {src}: {e}',
                          file=sys.stderr)
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='AN experiment batch orchestrator')
    parser.add_argument(
        '--config', required=True,
        help='Path to experiments.yaml')
    parser.add_argument(
        '--output-root', required=True,
        help='Root directory for organized NPZ outputs')
    parser.add_argument(
        '--ids', default=None,
        help='Comma-separated list of experiment ids to run (default: all)')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Print plan and exit')
    parser.add_argument(
        '--no-skip-existing', dest='skip_existing', action='store_false',
        help='Re-run experiments even if a completed NPZ already exists '
             '(default: skip).')
    parser.set_defaults(skip_existing=True)
    args = parser.parse_args(argv)

    cfg = import_yaml(args.config)
    defaults = cfg.get('defaults', {})
    experiments = cfg['experiments']

    if args.ids:
        wanted = set(s.strip() for s in args.ids.split(','))
        experiments = [e for e in experiments if e['id'] in wanted]
        if not experiments:
            print('No matching experiments', file=sys.stderr)
            return 1

    # Filter out already-completed experiments (skip-existing default ON)
    skipped = []
    pending = []
    for e in experiments:
        existing = already_completed(args.output_root, e) if args.skip_existing else None
        if existing is not None:
            skipped.append((e['id'], existing))
        else:
            pending.append(e)

    if skipped:
        print(f'Skipping {len(skipped)} already-completed experiments:')
        for eid, path in skipped:
            print(f'  {eid:20s} -> {path}')
    print(f'Plan: {len(pending)} experiments to run')
    for e in pending:
        zdes = f" z_des={e['z_des']}" if 'z_des' in e else ''
        plateau = ' [no-plateau]' if e.get('disable_plateau') else ''
        print(f"  {e['id']:20s} {e['mode']:14s} d={e['d']:>6} "
              f"start=({e['start']['x']:>7.0f},{e['start']['y']:>7.0f}) "
              f"max_steps={e.get('max_steps', defaults['max_steps']):>4}"
              f"{zdes}{plateau}")
    if args.dry_run:
        return 0
    if not pending:
        print('Nothing to run.')
        return 0
    experiments = pending

    rclpy.init()
    orch = Orchestrator()

    failures = []
    aborted_idx = None
    try:
        for idx, expt in enumerate(experiments):
            if not rclpy.ok():
                print(f'\n[ABORT] rclpy context invalid; '
                      f'stopping batch at index {idx}', file=sys.stderr)
                aborted_idx = idx
                break
            try:
                ok = run_one_experiment(orch, expt, defaults, args.output_root)
            except RCLError as e:
                print(f'\n[ABORT] RCL error during {expt["id"]}: {e}',
                      file=sys.stderr)
                # Plotter is in its own session and survives Ctrl+C; reap it so
                # the aborted run does not leave an orphan holding the output dir.
                kill_plotter_group(getattr(orch, '_active_plotter', None))
                orch._active_plotter = None
                failures.append(expt['id'])
                aborted_idx = idx
                break
            except Exception as e:  # noqa: BLE001 - keep batch alive on unexpected
                print(f'\n[FAIL] {expt["id"]}: {type(e).__name__}: {e}',
                      file=sys.stderr)
                # Same: this branch continues to the next run, so reap the
                # current plotter here or it accumulates one orphan per failure.
                kill_plotter_group(getattr(orch, '_active_plotter', None))
                orch._active_plotter = None
                failures.append(expt['id'])
                continue

            if not ok:
                failures.append(expt['id'])
            # Reset to NEUTRAL between runs (best-effort)
            try:
                if rclpy.ok():
                    orch.stop_pub.publish(Bool(data=True))
            except RCLError:
                pass
            time.sleep(2.0)
    finally:
        # Final safety net for any exit path (incl. KeyboardInterrupt, which
        # bypasses the per-run `except Exception`): never leave a live plotter.
        kill_plotter_group(getattr(orch, '_active_plotter', None))
        try:
            orch.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    remaining = []
    if aborted_idx is not None:
        remaining = [e['id'] for e in experiments[aborted_idx + 1:]]

    if failures or remaining:
        if failures:
            print(f'\n[FAIL] {len(failures)} experiments failed: {failures}',
                  file=sys.stderr)
        if remaining:
            print(f'[REMAINING] {len(remaining)} experiments not started '
                  f'(use --ids to resume):\n  --ids {",".join(remaining)}',
                  file=sys.stderr)
        return 1
    print(f'\n[OK] All {len(experiments)} experiments completed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
