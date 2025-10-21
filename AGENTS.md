# Repository Guidelines

## Project Structure & Module Organization
This ROS 2 workspace keeps source packages in `src/`. Core navigation nodes live in `src/adaptive_nav/` and `src/controller/`. Custom interfaces reside in `src/adaptive_navigation_interfaces/`, `src/pioneer_interfaces/`, and `src/robot_interfaces/`, so update their `msg/` or `srv/` trees whenever contracts change. Simulation models and world assets sit in `src/sensor_field_sim_2d/` and `src/rover_description/`, while reusable launch bundles are grouped in `src/base_launch/` and `src/sim_launch/`. Operator tooling sits in `src/teleop_core/`, with supporting utilities in `src/adaptive_navigation_utilities/` and `src/cluster_node/`. Build products stay in `build/` and `install/`, and logs accumulate in `log/`.

## Build, Test, and Development Commands
- `colcon build --symlink-install`: compile every package and keep Python entry points editable.
- `source install/setup.bash`: refresh your ROS 2 environment after each build or terminal session.
- `colcon test --event-handlers console_direct+`: run ament linters, interface checks, and package tests with plain output.
- `pytest src/adaptive_nav/test`: execute navigation unit tests quickly without a full colcon cycle.
- `ros2 launch sim_launch AN.launch.py use_sim_time:=true`: start the reference Gazebo scenario for local validation.

## Coding Style & Naming Conventions
Python follows ROS 2 norms: four-space indentation, snake_case modules, and entry points defined in `setup.py`. Launch files stay lowercase with underscores, while URDF, Xacro, and YAML assets use two-space indentation. Run `python3 -m ament_flake8 <package>` or rely on `colcon test` to enforce linting before pushing. Define new messages with CamelCase type names and uppercase constants to match existing interfaces.

## Testing Guidelines
Each package owns a `test/` directory; keep filenames `test_<feature>.py` and prefer deterministic fixtures. Mock serial, GPS, and radio interfaces so suites stay hardware-agnostic, and expose random seeds in algorithms to keep CI reproducible.

## Commit & Pull Request Guidelines
Write imperative commit summaries using the `type: description` pattern, e.g., `fix: guard null robot pose`. Group related changes together and rerun `colcon build` plus `colcon test --packages-select <pkg>` before pushing. Pull requests should summarise behaviour changes, list verification commands, and link tracking issues. Include screenshots or simulation recordings when altering operator dashboards or launch behaviour, and tag maintainers for every package you touched.

## Simulation & Configuration Tips
Reuse launch presets such as `ros2 launch base_launch cluster_hw_with_desired.launch.py` for hardware-in-the-loop tests, toggling behaviour through launch arguments instead of code edits. Store tuning parameters under each package `config/` directory—for example `src/teleop_core/config/joystick.yaml`. Register models in `rover_description` and expose them through `sim_launch` arguments so scenarios stay reproducible.
