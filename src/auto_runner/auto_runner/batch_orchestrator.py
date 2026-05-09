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
        self.config_pub = self.create_publisher(String, '/auto/config', 10)
        self.start_pub = self.create_publisher(Bool, '/auto/start', 10)
        self.stop_pub = self.create_publisher(Bool, '/auto/stop', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/rviz/pose2D', 10)

        self._leader_xy = None
        self._leader_history = collections.deque(maxlen=2000)
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

    def wait_for_leader(self, timeout_s: float = 30.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline and self._leader_xy is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._leader_xy is not None

    def reset_history(self):
        self._leader_history.clear()
        self._leader_xy = None
        self._oob_streak = 0
        for k in self._robot_z:
            self._robot_z[k] = None

    def push_history(self):
        if self._leader_xy is not None:
            self._leader_history.append(self._leader_xy)

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


def spawn_plotter(plotter_dir: str, sample_interval: float) -> subprocess.Popen:
    """Spawn trajectory_plotter_3d in its own process group so SIGINT is isolated."""
    os.makedirs(plotter_dir, exist_ok=True)
    args = [
        'ros2', 'run', 'sensor_field', 'trajectory_plotter_3d',
        '--ros-args',
        '-p', f'output_dir:={plotter_dir}',
        '-p', f'trajectory_sample_interval:={sample_interval}',
        '-p', 'visualization_mode:=contour',  # avoid blocking 3D show
    ]
    return subprocess.Popen(
        args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop_plotter(proc: subprocess.Popen, save_timeout_s: float = 30.0):
    """Send SIGINT to plotter process group; wait for clean exit (NPZ save)."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=save_timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5.0)


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

    if not expt.get('disable_plateau', defaults.get('disable_plateau', False)):
        win = expt.get('plateau_window_steps', defaults['plateau_window_steps'])
        eps = expt.get('plateau_eps', defaults['plateau_eps'])
        if len(orch._leader_history) >= win:
            recent = list(orch._leader_history)[-win:]
            xs = [p[0] for p in recent]
            ys = [p[1] for p in recent]
            span = max(max(xs) - min(xs), max(ys) - min(ys))
            if span < eps:
                return True, f'plateau (span={span:.3f} < {eps})'

    # Out-of-area: all 5 robots below sensor threshold for sustained period
    oob_win = expt.get('oob_window_steps', defaults['oob_window_steps'])
    if orch._oob_streak >= oob_win:
        thr = expt.get('oob_threshold', defaults['oob_threshold'])
        return True, (f'out-of-area (all robots z<{thr} '
                      f'for {orch._oob_streak} steps)')

    return False, ''


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

    # 2. Spawn the trajectory plotter
    shutil.rmtree(PLOTTER_OUTPUT_DIR, ignore_errors=True)
    os.makedirs(PLOTTER_OUTPUT_DIR, exist_ok=True)
    t_spawn = time.time()
    plotter = spawn_plotter(PLOTTER_OUTPUT_DIR, sample_interval)
    print(f'[{eid}] plotter spawned (pid={plotter.pid})')

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
    last_sample_t = time.time()
    progress_every = expt.get('progress_every_steps',
                              defaults.get('progress_every_steps', 20))
    while True:
        rclpy.spin_once(orch, timeout_sec=0.05)
        # Sample at the same interval as the plotter so step_count tracks NPZ rows
        if time.time() - last_sample_t >= sample_interval:
            orch.push_history()
            oob_thr = expt.get('oob_threshold', defaults['oob_threshold'])
            orch.push_oob_check(oob_thr)
            step_count += 1
            last_sample_t = time.time()

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
            print(f'[{eid}] terminating: {reason} (step={step_count})')
            break

    # 7. Stop navigation, cool down, signal plotter
    orch.stop_pub.publish(Bool(data=True))
    cool_deadline = time.time() + cooldown
    while time.time() < cool_deadline:
        rclpy.spin_once(orch, timeout_sec=0.05)
    print(f'[{eid}] SIGINT plotter ...')
    stop_plotter(plotter)

    # 8. Move NPZ to organized location with experiment id prefix
    npz_path = find_latest_npz(PLOTTER_OUTPUT_DIR, t_spawn)
    if npz_path is None:
        print(f'[{eid}] ERROR: no NPZ produced', file=sys.stderr)
        return False

    out_dir = os.path.join(output_root, expt['output_subdir'])
    os.makedirs(out_dir, exist_ok=True)
    # Prefix the experiment id so resume / skip-existing can match it later
    base = os.path.basename(npz_path)
    dest = os.path.join(out_dir, f'{eid}_{base}')
    shutil.move(npz_path, dest)
    print(f'[{eid}] saved -> {dest}')
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
                failures.append(expt['id'])
                aborted_idx = idx
                break
            except Exception as e:  # noqa: BLE001 - keep batch alive on unexpected
                print(f'\n[FAIL] {expt["id"]}: {type(e).__name__}: {e}',
                      file=sys.stderr)
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
