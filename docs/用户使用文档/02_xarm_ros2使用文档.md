# xarm_ros2 模块使用指南

---

## 目录

1. [启动前准备](#1-启动前准备)
2. [启动命令](#2-启动命令)
3. [常用 launch 参数详解](#3-常用-launch-参数详解)
4. [启动后状态检查](#4-启动后状态检查)
5. [运动控制命令](#5-运动控制命令)
6. [常见问题排查](#6-常见问题排查)

---

## 1. 启动前准备

### 1.1 脚本位置

```
publish/modules/src/xarm_ros2/start_xarm_ros2.sh
```

### 1.2 编译（如果没有预编译产物）

如果你拿到的是源码包，需要先编译：

```bash
cd /path/to/publish/modules
source /opt/ros/humble/setup.bash
export XARM_SDK_ROOT=/path/to/publish/xarm_can/package

colcon build \
  --base-paths src/xarm_ros2 src/xarm_description \
  --cmake-args -DXARM_SDK_ROOT=$XARM_SDK_ROOT
```

编译成功后会生成 `publish/modules/install/` 目录。

### 1.3 CAN 接口准备

```bash
# 配置 CAN 接口
cd /path/to/publish/xarm_can/package/libexec
bash setup_can_interfaces.sh

# 验证接口已 UP
ip link show | grep can
```

编译成功后会生成 `publish/modules/install/` 目录。

### 1.4 进入脚本目录

```bash
cd /path/to/publish/modules/src/xarm_ros2
```

---

## 2. 启动命令

### 2.1 脚本语法

```bash
./start_xarm_ros2.sh [single|bimanual] [ros2_launch_参数...]
```

第一个参数为模式：`single`（单臂）或 `bimanual`（双臂）。后续参数全部透传到 `ros2 launch`。

### 2.2 常用启动示例

```bash
# 1. 单臂默认启动
./start_xarm_ros2.sh

# 2. 单臂 + 假硬件（不接真实机械臂，用于软件验证）
./start_xarm_ros2.sh single use_fake_hardware:=true

# 3. 单臂 + 启动 RViz 可视化
./start_xarm_ros2.sh single start_rviz:=true

# 4. 单臂 + 指定 CAN 接口
./start_xarm_ros2.sh single can_interface:=can0

# 5. 单臂完整参数
./start_xarm_ros2.sh single \
  arm_type:=v10 \
  use_fake_hardware:=false \
  can_interface:=can0 \
  start_rviz:=true \
  robot_controller:=joint_trajectory_controller

# 6. 双臂默认启动
./start_xarm_ros2.sh bimanual

# 7. 双臂 + 假硬件 + RViz
./start_xarm_ros2.sh bimanual use_fake_hardware:=true start_rviz:=true

# 8. 查看帮助
./start_xarm_ros2.sh --help
```

---

## 3. 常用 launch 参数详解

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_type` | `v10` | 机械臂型号（v10 代表 10 自由度版本，x7 代表 7 自由度版本）|
| `use_fake_hardware` | `false` | `true` 使用虚拟硬件，无需实体机械臂，可用于纯软件调试 |
| `start_rviz` | `false` | `true` 同时启动 RViz 可视化工具，可以看到机械臂 3D 模型 |
| `can_interface` | `can0` | CAN 接口名称，必须与系统中实际接口名一致 |
| `robot_controller` | `joint_trajectory_controller` | 控制器类型，另可选 `forward_position_controller` |
| `description_package` | `xarm_description` | 机器人描述包，一般不需要改动 |
| `description_file` | `v10.urdf.xacro` | URDF 描述文件名 |
| `runtime_config_package` | `xarm_bringup` | 控制器配置包，存放控制器参数文件 |
| `arm_prefix` | （空）| 关节名前缀，双臂时通常为 `left_` 或 `right_` |
| `controllers_file` | `xarm_v10_controllers.yaml` | 控制器配置文件名 |

**两种控制器的区别：**

| 控制器 | 特点 | 适合场景 |
|--------|------|---------|
| `joint_trajectory_controller` | 按时间轨迹跟踪，精确平滑 | 精确运动、演示、录制数据 |
| `forward_position_controller` | 直接下发位置命令，响应快 | 快速验证、遥操作配合 |

---

## 4. 启动后状态检查

启动成功后，可以通过以下命令验证各组件运行状态：

```bash
# 查看已运行的 ROS2 节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看所有服务
ros2 service list

# 查看控制器状态（核心检查项）
ros2 control list_controllers

# 查看硬件接口状态
ros2 control list_hardware_interfaces
```

**期望看到的控制器状态：**
```
joint_trajectory_controller  [active]
joint_state_broadcaster       [active]
```

---

## 5. 运动控制命令


### 5.1 单臂轨迹 action（推荐，更精确平滑）

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [xarm_joint1, xarm_joint2, xarm_joint3, xarm_joint4, xarm_joint5, xarm_joint6, xarm_joint7],
      points: [{
        positions: [0.0, -0.4, 0.0, 1.2, 0.0, 1.0, 0.0],
        time_from_start: {sec: 3}
      }]
    }
  }"
```

`time_from_start` 表示从当前位置运动到目标位置的时间（3 秒），控制器会自动规划插值轨迹。

### 5.2 查看关节状态

```bash
# 实时查看关节位置
ros2 topic echo /joint_states

# 切换控制器
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate forward_position_controller
```

---

## 6. 常见问题排查

### 6.1 提示 `install/setup.bash` 不存在

**原因：** 发布包未预置 `install` 目录，或者 colcon 编译尚未执行。

**解决：** 在 `publish/modules` 下先执行 colcon 编译：
```bash
cd /path/to/publish/modules
source /opt/ros/humble/setup.bash
export XARM_SDK_ROOT=/path/to/publish/xarm_can/package
colcon build --base-paths src/xarm_ros2 src/xarm_description \
  --cmake-args -DXARM_SDK_ROOT=$XARM_SDK_ROOT
```

### 6.2 控制器未激活（active）

**解决：**
```bash
# 查看控制器状态
ros2 control list_controllers

# 手动激活控制器
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller active
```

### 6.3 真实硬件无动作

**排查步骤：**
1. 检查 `use_fake_hardware` 是否误设为 `true`
2. 检查 CAN 接口是否正常：`ip link show | grep can`
3. 检查控制器是否已激活：`ros2 control list_controllers`
4. 确认 `can_interface` 参数与实际接口名一致

### 6.4 colcon 编译时报"找不到包"

**原因：** 未使用 `--base-paths` 限制编译范围，colcon 扫描到了其他目录下同名包。

**解决：** 确保使用 `--base-paths src/xarm_ros2 src/xarm_description` 限制范围：
```bash
colcon build --base-paths src/xarm_ros2 src/xarm_description \
  --packages-select xarm xarm_bringup xarm_hardware xarm_description \
  --cmake-args -DXARM_SDK_ROOT=$XARM_SDK_ROOT
```

---
