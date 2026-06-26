# io_teleop_bridge

将 TeleXperience 遥操作话题桥接到 **x_air_sdk** (xArm ROS2 控制器) 的 ROS2 包。

## 解决的问题

TeleXperience 发出的关节指令是 `sensor_msgs/JointState` 格式（带关节名），
而 x_air_sdk 的 `forward_position_controller` 接收 `std_msgs/Float64MultiArray`
（纯数值数组，按关节顺序），`joint_trajectory_controller` 接收
`FollowJointTrajectory` action。

这个桥接节点负责格式转换与话题路由，让两边能直接对话。

## 数据流

```
 TeleXperience                                io_teleop_bridge                          x_air_sdk (xArm)
 ─────────────                                ───────────────                          ────────────────

 /io_teleop/joint_cmd ─────────────────┐
 /io_teleop/target_joint_from_vr ──────┼──→ 解析关节名 ──→ 按控制器顺序排列 ──→ /xx_forward_position_controller/commands
                                       │                                          (或 trajectory action)
 /io_teleop/target_gripper_status ─────┘
                                       │
                                       └──→ 0~1 映射 ──→  GripperCommand action ──→ /xx_gripper_controller/gripper_cmd
```

## 前提条件

| 条件 | 说明 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 版本 | Humble |
| xArm 控制器 | 已编译并安装到 `publish/modules/install/` |
| 仿真或真机 | 已启动 xArm ROS2 launch（见 xarm_ros2 使用文档） |

## 编译

```bash
cd publish/modules
source /opt/ros/humble/setup.bash
export XARM_SDK_ROOT=../xarm_can/package

colcon build \
  --base-paths src/io_teleop_bridge \
  --packages-select io_teleop_bridge \
  --symlink-install
```

`--symlink-install` 让 Python 文件链接到源码目录，修改后无需重新编译。

## 运行

### 1. 启动 xArm 仿真（新终端）

```bash
cd publish/modules/src/xarm_ros2

# 双臂仿真（推荐）
./start_xarm_ros2.sh bimanual use_fake_hardware:=true start_rviz:=true

# 或单臂仿真
./start_xarm_ros2.sh single use_fake_hardware:=true start_rviz:=true
```

等待输出中出现控制器激活信息，确认关节话题已发布。

### 2. 启动桥接节点（新终端）

```bash
cd publish/modules
source install/setup.bash

# 方式 A：forward 模式（默认，直接位置控制，响应最快）
ros2 launch io_teleop_bridge io_teleop_bridge.launch.py mode:=forward

# 方式 B：trajectory 模式（轨迹插值，运动更平滑）
ros2 launch io_teleop_bridge io_teleop_bridge.launch.py mode:=trajectory

# 方式 C：ros2 run
ros2 run io_teleop_bridge bridge_node
```

启动后应有日志：

```
[INFO] [io_teleop_bridge]: 桥接模式: forward
[INFO] [io_teleop_bridge]: io_teleop_bridge 已启动 (mode=forward, rate=30.0Hz)
[INFO] [io_teleop_bridge]: 订阅: /io_teleop/joint_cmd, /io_teleop/target_joint_from_vr, /io_teleop/target_gripper_status
```

### 3. 根据模式切换控制器

桥接节点只负责转发，**不自动切换控制器**。请先确认使用哪个控制器：

**forward 模式：** 需要 `forward_position_controller` 激活

```bash
# 双臂
ros2 control switch_controllers \
  --deactivate left_joint_trajectory_controller \
  --activate left_forward_position_controller
ros2 control switch_controllers \
  --deactivate right_joint_trajectory_controller \
  --activate right_forward_position_controller

# 单臂
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate forward_position_controller
```

**trajectory 模式：** `joint_trajectory_controller` 默认就是激活的，无需切换

```bash
# 验证
ros2 control list_controllers
# 应看到 ...joint_trajectory_controller [active]
```

## 测试方法

### 方法一：手动发关节指令

**双臂测试：**

```bash
# 左臂 joint1 和 joint2 分别转到 0.5rad 和 -0.3rad
ros2 topic pub /io_teleop/joint_cmd sensor_msgs/JointState \
  "{name: ['xarm_left_joint1', 'xarm_left_joint2'], position: [0.5, -0.3]}"
```

**预期结果：** RViz 中的左臂前两个关节转动。

---

**完整 7 关节 + 双臂：**

```bash
ros2 topic pub /io_teleop/joint_cmd sensor_msgs/JointState "{
  name: [
    'xarm_left_joint1', 'xarm_left_joint2', 'xarm_left_joint3',
    'xarm_left_joint4', 'xarm_left_joint5', 'xarm_left_joint6',
    'xarm_left_joint7',
    'xarm_right_joint1', 'xarm_right_joint2', 'xarm_right_joint3',
    'xarm_right_joint4', 'xarm_right_joint5', 'xarm_right_joint6',
    'xarm_right_joint7'
  ],
  position: [
    0.5, -0.3,  0.0,  1.2,  0.0,  0.5,  0.0,
   -0.5, -0.3,  0.0,  1.2,  0.0,  0.5,  0.0
  ]
}"
```

**预期结果：** 双臂在 RViz 中对称运动。

---

**单臂测试：**

使用不带 `left`/`right` 前缀的关节名：

```bash
ros2 topic pub /io_teleop/joint_cmd sensor_msgs/JointState "{
  name: ['xarm_joint1', 'xarm_joint2', 'xarm_joint3'],
  position: [0.5, -0.4, 0.0]
}"
```

---

### 方法二：发布循环指令（连续运动）

```bash
# 让左臂 joint1 在 0 ~ 1.0 rad 之间来回摆动
ros2 topic pub --rate 5 /io_teleop/joint_cmd sensor_msgs/JointState \
  "{name: ['xarm_left_joint1'], position: [1.0]}"
```

改发 `position: [0.0]` 就能回到原点。

---

### 方法三：测试夹爪

```bash
# 张开（0.0 = 张开）
ros2 topic pub --once /io_teleop/target_gripper_status sensor_msgs/JointState \
  "{position: [0.0, 0.0]}"

# 合拢（1.0 = 合拢）
ros2 topic pub --once /io_teleop/target_gripper_status sensor_msgs/JointState \
  "{position: [1.0, 1.0]}"
```

**预期结果：** RViz 中的两个夹爪开合。

---

### 方法四：用 VR 话题测试

```bash
# VR 话题和 joint_cmd 走同一个回调，效果完全一样
ros2 topic pub /io_teleop/target_joint_from_vr sensor_msgs/JointState \
  "{name: ['xarm_left_joint1'], position: [1.0]}"
```

---

### 方法五：完整流水线验证

按以下顺序在终端中操作：

```bash
# 终端 1：启动仿真
cd publish/modules/src/xarm_ros2
./start_xarm_ros2.sh bimanual use_fake_hardware:=true start_rviz:=true

# 终端 2：启动桥接（forward 模式）
cd publish/modules
source install/setup.bash
ros2 launch io_teleop_bridge io_teleop_bridge.launch.py mode:=forward

# 终端 3：切换控制器
ros2 control switch_controllers \
  --deactivate left_joint_trajectory_controller \
  --activate left_forward_position_controller
ros2 control switch_controllers \
  --deactivate right_joint_trajectory_controller \
  --activate right_forward_position_controller

# 终端 4：发送控制指令
# 左臂抬起
ros2 topic pub --rate 10 /io_teleop/joint_cmd sensor_msgs/JointState "{
  name: ['xarm_left_joint1','xarm_left_joint2','xarm_left_joint3',
         'xarm_left_joint4','xarm_left_joint5','xarm_left_joint6',
         'xarm_left_joint7',
         'xarm_right_joint1','xarm_right_joint2','xarm_right_joint3',
         'xarm_right_joint4','xarm_right_joint5','xarm_right_joint6',
         'xarm_right_joint7'],
  position: [0.0,-0.4,0.0,1.2,0.0,1.0,0.0,
             0.0,-0.4,0.0,1.2,0.0,1.0,0.0]
}"

# 然后张开夹爪
ros2 topic pub --once /io_teleop/target_gripper_status sensor_msgs/JointState \
  "{position: [0.0, 0.0]}"
```

---

### 方法六：用 Python 脚本测试

如果装了 `rclpy`，可以用脚本持续发送运动指令：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__("test_publisher")
        self.pub = self.create_publisher(JointState, "/io_teleop/joint_cmd", 10)
        self._timer = self.create_timer(0.1, self.timer_cb)
        self._t = 0.0

    def timer_cb(self):
        self._t += 0.1
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["xarm_left_joint1", "xarm_left_joint2", "xarm_left_joint3"]
        msg.position = [
            0.5 * math.sin(self._t),     # joint1 正弦摆动
           -0.4 + 0.2 * math.sin(self._t * 0.5),  # joint2 微动
            0.0,
        ]
        self.pub.publish(msg)

rclpy.init()
node = TestPublisher()
rclpy.spin(node)
```

保存为 `test_bridge.py`，直接运行：

```bash
python3 test_bridge.py
```

观察 RViz 中左臂前三个关节是否按正弦规律运动。

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `"forward"` | 桥接模式：`forward` (直接位置) 或 `trajectory` (轨迹插值) |
| `rate` | `30.0` | 定时刷新频率 (Hz)，仅 forward 模式有效 |
| `trajectory_time` | `0.5` | trajectory 模式的 `time_from_start` (秒)，越大运动越平滑但越慢 |

## 支持的遥操作话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/io_teleop/joint_cmd` | `sensor_msgs/JointState` | 关节位置指令 |
| `/io_teleop/target_joint_from_vr` | `sensor_msgs/JointState` | VR 遥操关节指令 |
| `/io_teleop/target_gripper_status` | `sensor_msgs/JointState` | 夹爪开合指令 (0~1) |

## 桥接的目标控制器

| 模式 | 目标话题/Action | 控制器 |
|------|----------------|--------|
| forward (单臂) | `/forward_position_controller/commands` | `forward_position_controller` |
| forward (左臂) | `/left_forward_position_controller/commands` | `left_forward_position_controller` |
| forward (右臂) | `/right_forward_position_controller/commands` | `right_forward_position_controller` |
| trajectory (单臂) | `/joint_trajectory_controller/follow_joint_trajectory` | `joint_trajectory_controller` |
| trajectory (左臂) | `/left_joint_trajectory_controller/follow_joint_trajectory` | `left_joint_trajectory_controller` |
| trajectory (右臂) | `/right_joint_trajectory_controller/follow_joint_trajectory` | `right_joint_trajectory_controller` |
| 夹爪 (单臂) | `/gripper_controller/gripper_cmd` | `gripper_controller` |
| 夹爪 (左) | `/left_gripper_controller/gripper_cmd` | `left_gripper_controller` |
| 夹爪 (右) | `/right_gripper_controller/gripper_cmd` | `right_gripper_controller` |

## 常见问题

### 桥接启动后日志说 "未响应"

某些 action server 可能还没准备好，不影响运行。只要模式对应的那个 server 起来了即可。

### 发送关节指令后机器人不动

检查顺序：

1. 确认启用了正确的控制器：
   ```bash
   ros2 control list_controllers
   ```
2. forward 模式需要用 `switch_controllers` 切换到 forward controller
3. 确认关节名拼写正确：`xarm_left_joint1` 不是 `xarm_joint1_left`
4. 检查桥接日志是否提示 mode_detected 了

### 只发了一个关节，其他关节会怎样

- **forward 模式：** 其他关节保持当前位置（bridge 只发当前消息中包含的关节，不包含的关节不受影响）
- **trajectory 模式：** 需要全部 7 个关节都到齐才发送，不足则跳过

### 夹爪值相反（0=合拢，1=张开）

这是 TeleXperience 的约定，和 controller 的约定刚好相反。程序中已经按 TeleXperience 的习惯：
`0 = 张开，1 = 合拢`。如果需要反过来，修改 `bridge_node.py` 中 `_send_gripper_goal` 调用处改为
`1.0 - position`。
