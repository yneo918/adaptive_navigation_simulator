# Adaptive Navigation Simulator

Adaptive Navigation Simulator is a ROS 2 Jazzy workspace that emulates a five-rover Pioneer cluster together with operator tooling, controller logic, and Gazebo-ready assets. The packages in `src/` cover controller logic, teleoperation, cluster coordination, fake hardware interfaces, and simulation launch files so that researchers can exercise formation algorithms without physical robots.

## Workspace Layout
- `src/adaptive_nav` – adaptive navigation algorithms and Python nodes
- `src/controller` – cluster controller orchestrating velocity and pose targets
- `src/cluster_node` – formation geometries and state machines
- `src/fake_rover_state_controller` – simulated hardware interfaces including the new `sim_rover` dynamic model
- `src/rover_description` – URDF/Xacro models, RViz configurations, and visualization launch files
- `src/sim_launch` – convenience launch descriptions for multi-rover simulation scenarios
- `src/teleop_core` – joystick GUI, command demultiplexer, and operator-facing utilities
- `src/sensor_field` – CSV-driven sensor field publisher and query services

## Quick Start
```bash
# Build and source
colcon build --symlink-install
source install/setup.bash

# Launch the five-rover simulation with Teleop GUI and RViz
ros2 launch sim_launch 5cluster.launch.py
```
The launch file spins up `controller`, `teleop_core`, five `sim_rover` instances, RViz, and the Pioneer visualizations required for a simulated cluster run.

## CSV Sensor Field Node
The `sensor_field` package provides a reusable node that turns CSV data into a `PointCloud2` stream and exposes lookup services.

```bash
ros2 run sensor_field csv_sensor_field \
  --ros-args \
  -p csv_path:=/absolute/path/to/data.csv \
  -p column_x:=latitude_column \
  -p column_y:=longitude_column \
  -p column_data:=value_column \
  -p interpret_latlon:=true \
  -p distance_scale:=1.0
```

### Key parameters
- `csv_path` (string, required): Absolute path to the CSV file. The file must include a header row.
- `column_x`, `column_y`, `column_z`, `column_data` (string): Column names for the spatial coordinates and the sensor value. Leave `column_z` empty for 2D datasets.
- `interpret_latlon` (bool, default `false`): Treat `column_x`/`column_y` as latitude and longitude (degrees) and project them into a local metric frame.
- `distance_scale` (float, default `1.0`): Multiplier applied to the projected north/east coordinates after conversion from latitude/longitude. Use `1.0` for meters, `0.001` for kilometers, or any custom scale.
- `frame_id`, `pointcloud_topic`, `publish_interval` (misc): Control the published frame, topic, and timer period.

When `interpret_latlon` is enabled, the first CSV row defines the local origin. The node always advertises the `get_sensor` service, dispatching to `GetSensor2D` or `GetSensor3D` based on the dataset dimensionality and returning the nearest stored value when an exact match is absent.

## Testing and Quality
Run the ROS 2 test suite with:
```bash
colcon test --event-handlers console_direct+
```
Individual algorithm packages (for example `adaptive_nav`) provide pytest suites under their `test/` folders for lightweight iterations.

## Contributing
Follow the guidelines in `AGENTS.md` for coding style, naming, testing expectations, and pull-request practices. Commit messages should use `type: description` (e.g. `feat: add formation parameter gui`) and include validation commands in every review.
