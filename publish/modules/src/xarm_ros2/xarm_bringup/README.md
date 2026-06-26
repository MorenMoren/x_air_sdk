# XArm Bringup

This package provides launch files to bring up the XArm robot system.

## Quick Start

Launch the XArm with v1.0 configuration and fake hardware:

```bash
ros2 launch xarm_bringup xarm.launch.py arm_type:=v10 hardware_type:=real
```

## Launch Files

- `xarm.launch.py` - Single arm configuration
- `xarm.bimanual.launch.py` - Dual arm configuration

## Key Parameters

- `arm_type` - Arm type (default: v10)
- `hardware_type` - Use real/mock/mujoco hardware (default: real)
- `can_interface` - CAN interface to use (default: can0)
- `robot_controller` - Controller type: `joint_trajectory_controller` or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization
