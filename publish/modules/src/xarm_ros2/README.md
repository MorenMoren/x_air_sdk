# XArm ROS 2 Integration

This package contains the core ROS 2 integration for VLAI XArm robotic arms, providing hardware abstraction, system bringup, and motion planning capabilities.

## Package Overview

### xarm
**Metapackage** that aggregates the core XArm ROS 2 packages.

- Aggregates: `xarm_bringup`, `xarm_description`, `xarm_hardware`
- Provides unified dependency management

### xarm_hardware
**ROS 2 Hardware Interface Plugin** for direct XArm control

**Features**:
- Implements `hardware_interface::SystemInterface` for ros2_control
- Real-time CAN communication with XArm controllers
- Support for XArm v10 with 7 joints and end-effector
- Configurable gripper (hand) support
- Joint state publishing and trajectory command handling
- Bimanual arm support with prefixed joint names

**Key Components**:
- `V10SimpleHardware` class: Main hardware interface implementation
- CAN interface abstraction for arm control
- Joint state feedback from arm sensors
- Trajectory execution via CAN protocol

**Configuration Parameters**:
- `can_interface`: CAN device to use (default: `can0`)
- `arm_prefix`: Prefix for joint names in bimanual setup (e.g., `left_`, `right_`)
- `hand`: Enable/disable gripper control (default: `true` for v10)

**Example Usage**:
```cpp
// Loaded as a plugin by hardware_interface
// Configuration in URDF:
<hardware>
  <plugin>xarm_hardware/V10SimpleHardware</plugin>
  <param name="can_interface">can0</param>
  <param name="arm_prefix">right_</param>
  <param name="hand">true</param>
</hardware>
```

### xarm_bringup
**System startup and ROS 2 controller management**

**Launch Files**:
- `xarm.launch.py`: Single arm configuration
- `xarm.bimanual.launch.py`: Dual arm bimanual setup

**Brings up**:
- Robot state publisher (from URDF)
- ros2_control hardware interface
- Controller manager
- Joint state broadcaster
- Selected robot controller

**Key Parameters**:
- `arm_type`: Arm model variant (default: `v10`)
- `hardware_type`: Hardware backend - `real`, `mock`, `mujoco` (default: `real`)
- `can_interface`: CAN interface device (default: `can0`)
- `robot_controller`: Motion controller type
  - `joint_trajectory_controller`: For trajectory-based control
  - `forward_position_controller`: For direct position commands
- `use_fake_hardware`: Use simulation instead of real hardware (default: `false`)

**Configuration Structure**:
```
config/
├── v10_controllers/
│   ├── xarm_v10_controllers.yaml              # Single arm config
│   ├── xarm_v10_bimanual_controllers.yaml     # Dual arm config
│   └── xarm_v10_bimanual_controllers_namespaced.yaml
```

**Quick Start**:
```bash
# Single arm with real hardware
ros2 launch xarm_bringup xarm.launch.py arm_type:=v10 

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{
  trajectory: {
    joint_names: ["xarm_joint1", "xarm_joint2", "xarm_joint3", "xarm_joint4", "xarm_joint5", "xarm_joint6", "xarm_joint7"],
    points: [{
      positions: [-0.0665674830243379, 0.1493476768139157, -0.2534905012588702, 0.8672846570534833, 0.014686808575570254, 0.12684061951628856, 0.8794918745708404],
      time_from_start: {sec: 4, nanosec: 0}
    }]
  }
}'

# Bimanual setup
ros2 launch xarm_bringup xarm.bimanual.launch.py arm_type:=v10 

# With fake hardware for testing
ros2 launch xarm_bringup xarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true
```


## Dependencies

### Build Dependencies
- `ament_cmake`: ROS 2 build system
- `hardware_interface`: ros2_control abstraction layer
- `pluginlib`: Plugin loading framework

### Runtime Dependencies
- `rclcpp`: ROS 2 C++ client library
- `controller_manager`: ros2_control manager
- `ros2_controllers`: Standard controller implementations
- `moveit_core`: Motion planning library
- `xarm_description`: Robot URDF definitions
- `xarm_can`: CAN communication library

### Optional Dependencies (MoveIt)
- `moveit_ros_move_group`: Motion planning service
- `moveit_kinematics`: IK solvers
- `moveit_planners`: Planning algorithms
- `moveit_ros_visualization`: RViz integration


## Declaration

This project is modified from [openarm](https://github.com/enactic/OpenArm), [mobile-aloha](https://github.com/MarkFzp/mobile-aloha), and [lerobot](https://github.com/huggingface/lerobot.git). We appreciate their open-source contributions.
