# OpenArm Workspace - Agent Development Guide

## Build Commands

```bash
# Build entire workspace
colcon build --symlink-install --packages-ignore openarm_hardware

# Build specific package
colcon build --packages-select <package_name>

# Source workspace after build
source install/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Test Commands

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# Run single test (pytest with marker)
colcon test --packages-select <package_name> --pytest-args -k <test_name>

# Run with verbose output
colcon test --event-handlers console_direct+
```

Example:
```bash
# Run flake8 lint tests only
colcon test --packages-select openarm_grasp_planner --pytest-args -m flake8

# Run pep257 docstring tests only
colcon test --packages-select openarm_vision --pytest-args -m pep257
```

## Code Style Guidelines

### Imports
- Group imports: standard library → third-party → ROS 2 → local
- Prefer explicit imports over `import rclpy` (use `from rclpy.node import Node`)
- Use 2-space indentation between import groups
- Keep imports at module top, except where needed for late imports in exception handlers

Example:
```python
import os
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
```

### Formatting
- Use pre-commit hooks (autopep8) for auto-formatting
- CMake files use cmake-format
- Keep lines reasonably concise (80-120 chars is typical in this codebase)
- Use spaces, not tabs

### Types
- Type hints recommended for new code (see openarm_mujoco_viewer style)
- Use type hints for function signatures, especially public APIs
- Return types should be explicit

Example:
```python
def timer_cb(self) -> None:
    """Timer callback."""
    pass

def compute_joint_angles(self, x: float, y: float) -> list[float]:
    """Compute joint angles from position."""
    return []
```

### Naming Conventions
- Classes: PascalCase (`BananaDetector`, `GraspPlanner`)
- Functions/methods: snake_case (`process`, `timer_callback`)
- Constants: UPPER_CASE (`MAX_JOINT_ANGLE`)
- Private members: single underscore prefix (`_mouse_left`)
- ROS 2 topics/services/actions: snake_case with forward slashes (`/joint_states`)
- TF frame names: snake_case with underscores (`openarm_body_link0`)

### Error Handling
- Use try/except for TF lookups, avoid crashing on transient errors
- Use ROS 2 logging for errors (`self.get_logger().error()`)
- Log warnings with `self.get_logger().warn()`
- Use debug logging for verbose info (`self.get_logger().debug()`)
- Catch specific exceptions where possible, use generic except sparingly

Example:
```python
try:
    transform = self.tf_buffer.lookup_transform(
        'openarm_body_link0', 'banana_target', rclpy.time.Time())
except tf2_ros.TransformException as ex:
    self.get_logger().error(f'TF lookup failed: {ex}')
    return None
```

### ROS 2 Patterns
- All nodes inherit from `rclpy.node.Node`
- Use `create_publisher()`, `create_subscription()`, `create_service()`, `create_timer()` for components
- Use `get_logger().info()` for important state changes
- Use `get_parameter()` for ROS 2 parameters
- Keep main() function pattern consistent:

```python
def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing
- Use pytest with markers: `@pytest.mark.flake8`, `@pytest.mark.pep257`, `@pytest.mark.linter`
- Test files go in `test/` directory
- Use `ament_flake8.main_with_errors()` for flake8 tests
- Use `ament_pep257.main()` for docstring tests

### Documentation
- Add docstrings for classes and public methods
- Keep comments minimal, prefer self-documenting code
- Chinese comments are acceptable in this codebase
- Coordinate system information should be documented (see grasp_planner.py)

### Coordinate Frames
- Base frame: `openarm_body_link0` (robot base center)
- Left arm base: `openarm_left_link0`
- Right arm base: `openarm_right_link0`
- Camera optical: `d435_optical_frame`
- Target frames: e.g., `banana_target` (published relative to base)

### Action Clients
- Use ActionClient for trajectory control (`control_msgs.action.FollowJointTrajectory`)
- Use ActionClient for MoveIt planning (`moveit_msgs.action.MoveGroup`)
- Check server readiness with `wait_for_server()`
- Use async callbacks to avoid blocking

### Console Scripts
Entry points are defined in setup.py:
```python
entry_points={
    'console_scripts': [
        'node_name = package.module:main',
    ],
}
```

Launch with: `ros2 run package_name node_name`

## Pre-commit Hooks
This repo uses pre-commit with:
- clang-format (C++/CMake)
- autopep8 (Python)

Install hooks: `pre-commit install`

## Running Nodes
```bash
# After sourcing workspace
ros2 run <package_name> <node_name>

# Example
ros2 run openarm_vision banana_detector
ros2 run openarm_grasp_planner grasp_planner

# Launch files
ros2 launch <package_name> <launch_file>.py
```

## Package Structure
- `src/<package_name>/<package_name>/` - Python modules
- `src/<package_name>/launch/` - Launch files
- `src/<package_name>/test/` - Tests
- `setup.py` - Package definition with console_scripts
- `package.xml` - ROS 2 package manifest
