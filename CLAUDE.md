# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Test Commands

```bash
# Build workspace with editable Python entry points
colcon build --symlink-install

# Source environment after build (required for each new terminal)
source install/setup.bash

# Run full test suite with console output
colcon test --event-handlers console_direct+

# Test specific package
colcon test --packages-select <package_name>
pytest src/<package_name>/test

# Run linter on specific package
python3 -m ament_flake8 <package_name>

# Launch reference simulation
ros2 launch sim_launch AN.launch.py use_sim_time:=true

# Launch 5-robot scenarios
ros2 launch sim_launch AN_5.launch.py
ros2 launch sim_launch 5cluster.launch.py

# Launch controller only
ros2 launch sim_launch controller.launch.py

# Launch sensor field visualization
ros2 launch sim_launch sensor_field.launch.py

# Run teleoperation tools
ros2 run teleop_core run_joy_with_gui
ros2 run gui_package gui_adaptive_navigation
ros2 run virtual_joy virtual_joy
```

## Architecture Overview

### Multi-Robot Cluster Control System

This is a ROS 2 Jazzy workspace implementing adaptive navigation for a five-robot Pioneer cluster. The system uses a hierarchical control architecture where individual robot states are aggregated into cluster-space coordinates, enabling formation control and gradient-based navigation.

**Core Control Flow:**
1. **Robot State Publishing** (`fake_rover_state_controller`) publishes individual robot poses via `/prefix/robot_id/pose2D`
2. **Cluster Kinematics** (`cluster_node/Cluster.py`) transforms robot-space → cluster-space using symbolic Jacobians (SymPy)
3. **Controller** (`controller/Controller.py`) computes cluster velocity commands using PD control
4. **Inverse Kinematics** transforms cluster commands → individual robot velocities
5. **Adaptive Navigator** (`adaptive_nav`) computes gradient-based trajectories from sensor field data

**Coordinate System (REP103 Compliant):**
- X+ forward, Y+ left, Z+ up (right-handed)
- Counter-clockwise rotation positive around Z-axis
- See `docs/REP103_COMPLIANCE_PLAN.md` for detailed compliance status

### Package Responsibilities

**Navigation Stack:**
- `adaptive_nav/` — Gradient-based adaptive navigation node, scalar field gradient computation, control mode management
- `controller/` — Cluster controller with PD control, heading manager, transforms cluster commands to robot velocities
- `cluster_node/` — Symbolic kinematics (forward/inverse/Jacobian), formation geometries (triangle, pentagon), cluster state machine

**Simulation:**
- `fake_rover_state_controller/` — Simulated robot dynamics (`sim_rover.py`, `sim_robot_2d.py`), integrates velocity commands into pose updates
- `rover_description/` — URDF/Xacro robot models, RViz configurations, mesh assets
- `base_launch/` & `sim_launch/` — Launch file bundles for various scenarios

**Operator Tooling:**
- `teleop_core/` — Joystick GUI, command demultiplexer
- `gui_package/` — PyQt-based adaptive navigation control panel, shared `my_ros_module.py` helper
- `virtual_joy/` — Virtual joystick for scripted teleoperation

**Sensor Field System:**
- `sensor_field/` — CSV-driven sensor field publishers with interpolation (IDW, thin-plate spline), service-based spatial queries (`GetSensor2D`/`GetSensor3D`), lat/lon to Cartesian projection support

**Messaging:**
- `adaptive_navigation_interfaces/`, `pioneer_interfaces/`, `robot_interfaces/` — Custom message and service definitions

### Key Architecture Patterns

**Cluster Kinematics:**
- Uses SymPy to define symbolic forward/inverse kinematics and Jacobians
- Lambdified functions compiled at initialization for runtime performance
- Configuration classes inherit from `ClusterConfigurationBase` abstract class
- Formation types: `TriangleatCentroid`, `TriangleatLeader`, `PentagonatLeader`

**Control Modes:**
- Position control (`POS`): PD controller drives cluster to desired pose
- Velocity control (`VEL`): Direct velocity passthrough to robots
- Adaptive navigation: Gradient descent/ascent on sensor fields

**Sensor Field Pipeline:**
- CSV loader → lat/lon projection (optional) → interpolation grid generation → PointCloud2 publishing + service queries
- Supports caching of interpolated grids (pickle format) for faster startup
- Two interpolation methods: inverse-distance weighting (IDW) and thin-plate spline (TPS)

**Namespace Structure:**
- Robot topics: `/prefix/robot_id/{pose2D, sensor, cmd_vel}`
- Cluster topics: `/ctrl/{cmd_vel, adaptive_mode, cluster_mode}`
- Sensor field: `/sensor_field/{points, fill_points}` + service endpoints

## Important Development Notes

### When Modifying Robot Dynamics
All pose integration in `fake_rover_state_controller/` must follow REP103 conventions:
- `theta += angular_velocity * dt` (NOT `-=`)
- Forward motion: `x += velocity * cos(theta) * dt`, `y += velocity * sin(theta) * dt`
- See `docs/REP103_COMPLIANCE_PLAN.md` for files requiring coordinate transforms

### When Adding Cluster Formations
1. Create new configuration class inheriting `ClusterConfigurationBase`
2. Define symbolic forward/inverse kinematics in `setup_kinematics()`
3. Register in `Cluster.py` configuration factory
4. Update `cluster_params` expected length validation in `Controller.py`

### When Adding Sensor Field Types
1. Inherit from `CsvSensorFieldPublisher` or create standalone node
2. Publish `sensor_msgs/PointCloud2` with `x, y, z, intensity` fields
3. Provide `GetSensor2D` or `GetSensor3D` service for spatial queries
4. Update launch files to expose new parameters

### Python GUI Integration
All GUI packages use `gui_package/my_ros_module.py` for PyQt/ROS integration:
- `PubSubManager`: wraps ROS publishers/subscribers with Qt signal emissions
- Import as `from gui_package.my_ros_module import PubSubManager`
- Keep GUI event loop separate from ROS spin

### Launch File Conventions
- Hardware-in-the-loop launches: `base_launch/cluster_hw_with_desired.launch.py`
- Pure simulation launches: `sim_launch/AN*.launch.py`
- Use `use_sim_time:=true` for Gazebo scenarios
- Robot namespacing controlled by `prefix` parameter in launch files

## Coding Standards

**Style:**
- Python: 4-space indent, snake_case, ROS 2 naming conventions
- Launch files: lowercase with underscores (e.g., `sensor_field.launch.py`)
- URDF/YAML: 2-space indent

**Testing:**
- Package tests in `test/` directories
- Filename pattern: `test_<feature>.py`
- Use deterministic fixtures; expose random seeds for reproducibility
- Mock hardware interfaces (serial, GPS, radio)

**Commit Messages:**
- Format: `type: description` (e.g., `fix: guard null robot pose`)
- Types: feat, fix, refactor, test, docs
- Verify with `colcon build && colcon test --packages-select <pkg>` before pushing

**Pull Requests:**
- Summarize behavior changes
- List verification commands executed
- Include screenshots for GUI/visualization changes
- Tag maintainers for affected packages
