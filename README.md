# Adaptive Navigation Simulator

Adaptive Navigation Simulator is a ROS 2 Jazzy workspace for coordinating a five-rover Pioneer cluster, exercising adaptive navigation algorithms, and operating a simulated field through Gazebo. The repo couples navigation logic, operator tooling, fake hardware interfaces, and reusable launch descriptions so you can test formation behaviours end to end without physical robots.

## Highlights
- Multi-rover navigation stack with adaptive path planners, cluster coordination, and formation managers
- Operator tooling that covers joystick teleoperation, PyQt GUIs, and a virtual joystick for scripted inputs
- Simulation assets for Gazebo and RViz, including Pioneer URDF/Xacro models and visualization helpers
- Sensor-field utilities that stream CSV datasets, synthetic environments, and interpolation grids to the robot cluster
- Launch bundles for both pure simulation and hardware-in-the-loop validation

## Prerequisites
- ROS 2 Jazzy desktop installation (Gazebo and RViz included)
- Python 3.10+ with `colcon` and ROS build tools (`python3-colcon-common-extensions`)
- System dependencies sourced from `ros-jazzy-*` packages (e.g., `ros-jazzy-rviz2`, `ros-jazzy-tf2-ros`, `ros-jazzy-launch`)
- Optional Python extras from `requirements.txt` (`pip install -r requirements.txt`) for plotting, GUI, and math utilities
- A workspace with this repository checked out under `~/.../adaptive_navigation_simulator`

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
- Full simulation reference: `ros2 launch sim_launch AN.launch.py use_sim_time:=true`
- Alternate five-rover scenes: `ros2 launch sim_launch AN_5.launch.py` or `ros2 launch sim_launch 5cluster.launch.py`
- Hardware-in-the-loop baseline: `ros2 launch base_launch cluster_hw_with_desired.launch.py`
- Sensor field standalone visualisation: `ros2 launch sim_launch sensor_field.launch.py`
- Teleoperation tools: `ros2 run teleop_core run_joy_with_gui`, `ros2 run gui_package gui_adaptive_navigation`, or `ros2 run virtual_joy virtual_joy`

Each launch file wires together the navigation stack, fake rover drivers, RViz, and optional operator interfaces. Use `ros2 launch sim_launch controller.launch.py` to bring up only the controller layer when composing custom experiments.

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
- `src/teleop_core/` – joystick GUI, command demultiplexer, and ROS node wiring
- `src/gui_package/` – PyQt-based adaptive navigation control panel
- `src/virtual_joy/` – ROS 2 virtual joystick publisher for scripted teleoperation
- `src/lib/my_ros_module.py` – shared PyQt helper imported by GUI-oriented packages

### Messaging interfaces
- `src/adaptive_navigation_interfaces/`, `src/pioneer_interfaces/`, `src/robot_interfaces/` – custom `msg/` and `srv/` definitions that back the navigation and teleop contracts

### Sensor field toolkit
- `src/sensor_field/` – CSV-driven sensor publishers, 2D/3D interpolation utilities, and sample RF field nodes
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

Additional nodes such as `sensor_field_pose2d`, `rf_field`, and `topography_2d` generate synthetic data for testing without CSV files.

## Testing & Quality
- Run the full test suite: `colcon test --event-handlers console_direct+`
- Package-level checks: `pytest src/adaptive_nav/test` or `colcon test --packages-select <pkg>`
- Linting: rely on `colcon test` or execute `python3 -m ament_flake8 <package>` for targeted runs

Logs accumulate under `log/`, while build products live in `build/` and `install/`. Clear these directories between major upgrades if you hit stale artifacts.

## Contributing
Follow `AGENTS.md` for coding standards, naming, testing expectations, and pull-request practices. Group related changes into logical commits using the `type: description` format (for example, `fix: guard null robot pose`) and document the verification commands you ran. When altering launch behaviour or GUI tooling, include screenshots or simulation recordings to aid reviewers.
