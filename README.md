# Adaptive Navigation Simulator

Adaptive Navigation Simulator is a ROS 2 Jazzy workspace for coordinating a five-rover Pioneer cluster, exercising adaptive navigation algorithms, and operating a simulated field through Gazebo. The repo couples navigation logic, operator tooling, fake hardware interfaces, and reusable launch descriptions so you can test formation behaviours end to end without physical robots.

## Highlights
- Multi-rover navigation stack with adaptive path planners, cluster coordination, and formation managers
- Operator tooling including a PyQt adaptive-navigation GUI, RViz pose placement, and a virtual joystick for scripted inputs
- Simulation assets for Gazebo and RViz, including Pioneer URDF/Xacro models and visualization helpers
- Sensor-field utilities that stream CSV datasets, synthetic environments, and interpolation grids to the robot cluster
- Launch bundles for both pure simulation and hardware-in-the-loop validation

## Prerequisites
- ROS 2 Jazzy desktop installation (Gazebo and RViz included)
- Python 3.12 with `colcon` and ROS build tools (`python3-colcon-common-extensions`)
- System dependencies sourced from `ros-jazzy-*` packages (e.g., `ros-jazzy-rviz2`, `ros-jazzy-tf2-ros`, `ros-jazzy-launch`)
- A workspace with this repository checked out under `~/.../adaptive_navigation_simulator`

### Python dependencies (apt preferred)
ROS 2 Jazzy links against the apt-provided numpy (1.26). Installing numpy 2.x via pip causes binary-incompatibility errors (e.g., `numpy.dtype size changed`) for packages like scikit-image. Prefer apt for C-extension packages:

```bash
sudo apt install \
    python3-numpy python3-scipy python3-sympy python3-yaml \
    python3-matplotlib python3-pandas python3-skimage
```

PyQt6 is not in apt; install via pip for the adaptive navigation GUI:
```bash
pip install PyQt6
```

If you previously installed numpy/pandas/matplotlib/scikit-image via pip and hit ABI errors, remove the user-site copies so the apt versions are loaded:
```bash
pip uninstall -y numpy pandas matplotlib scikit-image
```

## Setup & Build
```bash
# Source the ROS 2 environment once per shell
source /opt/ros/jazzy/setup.bash

# Build the workspace and expose editable Python entry points
colcon build --symlink-install
source install/setup.bash
```
Re-run `source install/setup.bash` after each build or new terminal session so ROS 2 can discover the packages.

## Common Launch Scenarios
- **Cesium radiation field experiments (primary)**: `ros2 launch sim_launch AN_5_modular.launch.py field_type:=cesium`
- Alternate sensor fields via the same modular launch: `field_type:=paper`, `field_type:=disaster`, `field_type:=topography`
- Five-rover scenes with synthetic terrain: `ros2 launch sim_launch AN_5.launch.py`
- Three-rover reference: `ros2 launch sim_launch AN.launch.py use_sim_time:=true`
- Hardware-in-the-loop baseline: `ros2 launch base_launch cluster_hw_with_desired.launch.py`
- Sensor field standalone visualisation: `ros2 launch sim_launch sensor_field.launch.py`
- Standalone GUI / controller / virtual joystick: `ros2 run gui_package gui_adaptive_navigation`, `ros2 launch sim_launch controller.launch.py`, `ros2 run virtual_joy virtual_joy`

The modular launch (`AN_5_modular.launch.py`) composes four components via per-component toggles (`enable_controller`, `enable_sensor_field`, `enable_visualization`, `enable_robots`) and a `time_scale` argument. `field_type` selects the sensor field package: `cesium` is the cesium-137 CSV dataset with log10 normalization, `disaster` is the generic CSV publisher, `paper` is the paper_field analytic case, and `topography` uses the synthetic ridge/trench generator.

> Run launches from the workspace root. `cesium_field.yaml` uses a relative `csv_path: data/ces_radiation_field.csv` which is resolved from the current working directory.

## Experiment Workflow (cesium)
Typical flow for reproducing the experiments in `EXPERIMENT_PLAN.md` / `ADDITIONAL_EXPERIMENTS.md`:

```bash
# Terminal 1 — simulator + cesium field + GUI + RViz (all bundled)
cd ~/mlu/adaptive_navigation_simulator
source install/setup.bash
ros2 launch sim_launch AN_5_modular.launch.py field_type:=cesium

# Terminal 2 — trajectory recorder (saves NPZ + PNG on Ctrl+C)
cd ~/mlu/adaptive_navigation_simulator
source install/setup.bash
ros2 run sensor_field trajectory_plotter_3d --ros-args \
    -p output_dir:=Max10_new \
    -p num_robots:=5

# Terminal 3 — place the cluster at the experiment start pose
ros2 topic pub --once /rviz/pose2D geometry_msgs/msg/Pose2D \
    "{x: -950.0, y: 763.0, theta: 3.14}"
```

In the GUI (`adaptive_navigation_gui`):
1. Set cluster size `d` (10 or 100).
2. Select the adaptive mode (MAX / MIN / CROSSTRACK_CW / RIDGE_DOWN / TRENCH_UP).
3. For CROSSTRACK, set `z_des` (0.6 ≈ 483 kBq/m² in the normalized cesium field).
4. Switch `Cluster Mode` to `ADPTV_NAV_M` to engage adaptive navigation.

`trajectory_plotter_3d` subscribes to `/sensor_field/points` (terrain) and every `/<robot_id>/pose2D` (trajectories). On shutdown it writes `<timestamp>_trajectory_data.npz` and `<timestamp>_trajectory_3d_contour.png` under `output_dir/`.

## Workspace Layout
### Core navigation
- `src/adaptive_nav/` – adaptive navigation behaviours and gradient-based planners
- `src/controller/` – cluster controller, heading manager, and pose forwarding utilities
- `src/cluster_node/` – formation geometries, state machines, and shared cluster configuration
- `src/adaptive_navigation_utilities/` – reusable tools for namespace creation, live data capture, and plotting

### Simulation & visualization
- `src/fake_rover_state_controller/` – simulated rover dynamics, joint publishers, and fake sensor endpoints
- `src/rover_description/` – URDF/Xacro assets, RViz setups, and robot model resources
- `src/base_launch/` & `src/sim_launch/` – launch descriptions for reference scenarios and hardware bridges

### Operator tooling
- `src/gui_package/` – PyQt-based adaptive navigation control panel (`gui_adaptive_navigation`)
- `src/virtual_joy/` – ROS 2 virtual joystick publisher for scripted teleoperation
- `src/lib/my_ros_module.py` – shared PyQt/ROS helper imported by GUI-oriented packages via symlink (`src/<pkg>/<pkg>/my_ros_module.py → ../../lib/my_ros_module.py`)
- `src/teleop_core/` – legacy joystick GUI / command demux; currently `COLCON_IGNORE`-excluded from builds and not used by the default workflows

### Messaging interfaces
- `src/adaptive_navigation_interfaces/`, `src/pioneer_interfaces/`, `src/robot_interfaces/` – custom `msg/` and `srv/` definitions that back the navigation and teleop contracts

### Sensor field toolkit
- `src/sensor_field/` – CSV-driven sensor publishers, 2D/3D interpolation utilities, contour visualizer, trajectory plotter, and sample RF field nodes
- `data/ces_radiation_field.csv` – cesium-137 deposition dataset used by `cesium_sensor_field`
- `data/radiation_field.csv` – example dataset for quick CSV field experiments

## Sensor Field Nodes
The `sensor_field` package provides flexible point cloud publishers and query services for environmental datasets.

### CSV-backed field
```bash
ros2 run sensor_field csv_sensor_field \
  --ros-args \
  -p csv_path:=$PWD/data/radiation_field.csv \
  -p column_x:=x \
  -p column_y:=y \
  -p column_data:=dose \
  -p interpret_latlon:=false \
  -p fill_method:=thin_plate
```
Key parameters:
- `csv_path` (string, required) – path to a CSV file with a header row
- `column_x`, `column_y`, `column_z`, `column_data` (string) – column names for spatial axes and sensor values
- `interpret_latlon` (bool, default `false`) – treat x/y columns as latitude/longitude degrees before projection
- `distance_scale` (float, default `1.0`) – scale factor applied after lat/lon conversion
- `fill_rect_min` / `fill_rect_max` (string) – comma-separated bounds for generating an interpolation grid
- `fill_method` (enum `idw` or `thin_plate`) – choose inverse-distance weighting or thin-plate spline interpolation
- `service_inputs_latlon` (bool, default `false`) – accept service queries in latitude/longitude when lat/lon mode is active

Additional parameters control frame naming, publication cadence, and interpolation density, including `frame_id`, `pointcloud_topic`, `publish_interval`, `fill_spacing`, `fill_radius`, `fill_neighbor_count`, `fill_weight_power`, and `fill_tps_regularization`.

The node publishes a `sensor_msgs/PointCloud2` stream and exposes `robot_interfaces/GetSensor2D` or `GetSensor3D` based on the dataset dimensionality. Interpolation results are optionally published on a secondary point cloud topic.

### Cesium field (log10-normalized)
`cesium_sensor_field` wraps `csv_sensor_field` with log10 normalization suited to cesium-137 deposition values (Bq/m²). It reads the same CSV format and exposes the same `/sensor_field/points` topic and `get_sensor` service, so downstream consumers (adaptive_nav via `fake_sensor`, `trajectory_plotter_3d`, `contour_visualizer`) work unchanged. Config: `src/sensor_field/config/cesium_field.yaml`.

### Other nodes
- `topography_2d` – synthetic ridge/trench terrain generator (used by default `AN_5.launch.py`)
- `paper_field`, `sample_2d` – analytic fields for paper-reproduction scenarios
- `sensor_field_pose2d`, `rf_field` – RF propagation models publishing to `/rf_field`
- `contour_visualizer` – draws equi-value contours from `/sensor_field/points` in RViz (requires `python3-skimage`)
- `trajectory_plotter_3d` – records robot trajectories + terrain snapshots to NPZ/PNG for post-processing

## Testing & Quality
- Run the full test suite: `colcon test --event-handlers console_direct+`
- Package-level checks: `pytest src/adaptive_nav/test` or `colcon test --packages-select <pkg>`
- Linting: rely on `colcon test` or execute `python3 -m ament_flake8 <package>` for targeted runs

Logs accumulate under `log/`, while build products live in `build/` and `install/`. Clear these directories between major upgrades if you hit stale artifacts.

## Contributing
Follow `AGENTS.md` for coding standards, naming, testing expectations, and pull-request practices. Group related changes into logical commits using the `type: description` format (for example, `fix: guard null robot pose`) and document the verification commands you ran. When altering launch behaviour or GUI tooling, include screenshots or simulation recordings to aid reviewers.
