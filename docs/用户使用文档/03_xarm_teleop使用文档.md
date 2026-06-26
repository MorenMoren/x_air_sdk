# xarm_teleop 遥操作模块使用指南

---

## 目录

1. [模块目录结构](#1-模块目录结构)
2. [运行前准备](#2-运行前准备)
3. [编译](#3-编译)
4. [运行](#4-运行)
5. [SDK 二次开发](#5-sdk-二次开发)
6. [常见问题排查](#6-常见问题排查)

---

## 1. 模块目录结构

`publish` 目录中，xarm_teleop 结构如下：

```text
publish/modules/src/xarm_teleop/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── xarm_teleop_sdk.h          ← 对外 C ABI 头文件
├── config/
│   ├── leader.yaml                ← Leader 控制参数（kp/kd/重力补偿系数）
│   └── follower.yaml              ← Follower 控制参数
├── control/
│   ├── xarm_unilateral_control.cpp       ← 单边遥操作主程序入口
│   └── xarm_unilateral_control_ros2.cpp  ← ROS2 版主程序入口
├── src/
│   └── xarm_teleop_sdk_impl.cpp   ← SDK 实现（允许用户重编译适配）
├── script/
│   ├── launch_unilateral.sh       ← 单边遥操作启动脚本
│   └── launch_unilateral_ros2.sh  ← ROS2 版启动脚本
└── prebuilt/
    └── xarm_teleop/lib/
        └── libxarm_teleop_lib.so  ← 预编译核心库（核心实现，不以源码形式暴露）
```

**说明**：
- `control/` 目录仅保留单边遥操作入口，作为 SDK 调用示例
- 核心控制逻辑封装在 `prebuilt/xarm_teleop/lib/libxarm_teleop_lib.so` 中
- `xarm_teleop_sdk.h` 是唯一对外接口声明，二次开发只依赖此头文件

---

## 2. 运行前准备

### 2.1 基础环境

| 要求 | 说明 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 版本 | Humble |
| ROS2 依赖 | `xacro`、`colcon`、`ros2_control` |
| CAN 接口 | 已完成物理接线和 SocketCAN 配置 |

### 2.2 CAN 接口准备

启动前，先确认 CAN 接口可用：

```bash
# 使用 xarm_can SDK 中的配置脚本一键初始化
cd /path/to/publish/xarm_can/package/libexec
bash setup_can_interfaces.sh

# 验证接口状态
ip link show | grep can
```

预期输出示例：

```text
can0: <NOARP,UP,LOWER_UP> mtu 72 qdisc pfifo_fast state UP mode DEFAULT
can1: <NOARP,UP,LOWER_UP> mtu 72 qdisc pfifo_fast state UP mode DEFAULT
```

---

## 3. 编译

### 3.1 编译命令

```bash
cd /path/to/publish/modules
source /opt/ros/humble/setup.bash
export XARM_SDK_ROOT=/path/to/publish/xarm_can/package

colcon build \
  --base-paths src/xarm_ros2 src/xarm_description src/xarm_teleop \
  --packages-select xarm xarm_bringup xarm_hardware xarm_description \
                   xarm_bimanual_moveit_config xarm_teleop \
  --cmake-args -DXARM_SDK_ROOT=$XARM_SDK_ROOT
```

---

## 4. 运行

### 4.1 单边遥操作

```bash
cd /path/to/publish/modules/src/xarm_teleop
bash start_xarm_teleop.sh unilateral right_arm
```

指定 CAN 接口：

```bash
bash start_xarm_teleop.sh unilateral right_arm can0 can2
bash start_xarm_teleop.sh unilateral left_arm  can1 can3
```

运行成功后的日志：

```text
========================================
 xarm_teleop 启动
========================================
  模式        : unilateral
  机械臂侧    : left_arm
  Leader CAN  : can0
  Follower CAN: can1
  工作空间    : /home/vlai/x_airv7/publish/modules
```

### 4.2 单边遥操作 ROS2 版

```bash
cd /path/to/publish/modules/src/xarm_teleop
bash start_xarm_teleop.sh unilateral_ros2 left_arm 
```

指定 CAN 接口：

```bash
bash start_xarm_teleop.sh unilateral_ros2 left_arm can0 can1
```

运行成功后的日志：

```text
[INFO] Launching unilateral control ROS2 node...
[INFO] [unilateral_control_ros2]: === XArm 单边遥操作 ROS2 (SDK 1.0.0) ===
[INFO] [unilateral_control_node]: 归位服务已就绪: /robot_go_home
[INFO] [unilateral_control_ros2]: 控制循环运行中，按 Ctrl+C 停止...
```

查看 ROS2 话题：

```bash
ros2 topic list
# 预期看到：
# /xarm_left/joint_states
# /xarm_right/joint_states

ros2 topic echo /xarm_left/joint_states
```

调用归位服务：

```bash
ros2 service call /robot_go_home std_srvs/srv/Trigger
```

### 4.3 两种启动方式区别

| 脚本 | 依赖 ROS2 | 输出 | 适用场景 |
|------|-----------|------|---------|
| `unilateral` | 否 | 控制台日志 | 链路验证、不需要 ROS2 集成 |
| `unilateral_ros2.sh` | 是 | `/joint_states` 话题 + `/robot_go_home` 服务 | 接入 ROS2 生态、lerobot 数据采集 |

如果只验证控制链路，优先用 `unilateral`；需要 lerobot_collector 采集数据时，使用 `unilateral_ros2.sh`。

---

## 5. SDK 二次开发

### 5.1 接口总览

| 类别 | 接口 |
|------|------|
| 会话创建 | `xarm_teleop_create_unilateral` / `xarm_teleop_create_bilateral` / `xarm_teleop_create_gravity_comp` |
| 生命周期 | `xarm_teleop_start` / `xarm_teleop_stop` / `xarm_teleop_wait` / `xarm_teleop_is_running` / `xarm_teleop_destroy` |
| 扩展能力 | `xarm_teleop_set_joint_state_callback` / `xarm_teleop_go_home` |
| 诊断 | `xarm_teleop_get_last_error` / `xarm_teleop_version` |

### 5.2 错误码

| 返回码 | 值 | 含义 |
|--------|-----|------|
| `XARM_TELEOP_OK` | `0` | 成功 |
| `XARM_TELEOP_ERR_GENERAL` | `-1` | 通用失败（内部线程异常）|
| `XARM_TELEOP_ERR_PARAM` | `-2` | 参数错误（空指针、非法 `arm_side`）|
| `XARM_TELEOP_ERR_INIT` | `-3` | 初始化失败（CAN 初始化、电机初始化）|
| `XARM_TELEOP_ERR_FILE` | `-4` | 文件不存在（URDF 路径或配置目录错误）|
| `XARM_TELEOP_ERR_RUNNING` | `-5` | 会话已在运行（重复调用 `start`）|

### 5.3 API 详细说明

#### 5.3.1 `xarm_teleop_create_unilateral` — 创建单边遥操作控制器

```c
int xarm_teleop_create_unilateral(
    const char*             can_leader,        // Leader 臂 CAN 接口，如 "can0"
    const char*             can_follower,      // Follower 臂 CAN 接口，如 "can2"
    const char*             leader_urdf_path,  // Leader 臂 URDF 文件绝对路径
    const char*             follower_urdf_path,// Follower 臂 URDF 文件绝对路径
    const char*             arm_side,          // "left_arm" 或 "right_arm"
    xarm_teleop_handle_t*   out                // 输出句柄
);
```

| 参数 | 说明 |
|------|------|
| `can_leader` | 操作者手持主臂的 CAN 接口 |
| `can_follower` | 跟随执行从臂的 CAN 接口 |
| `leader_urdf_path` | 用于计算主臂重力补偿的 URDF 模型 |
| `follower_urdf_path` | 用于计算从臂重力补偿的 URDF 模型 |
| `arm_side` | 影响重力补偿方向，只能是 `"left_arm"` 或 `"right_arm"` |
| `out` | 成功时写入有效句柄，失败时保持 `NULL` |

**语义**：Leader 施加重力补偿（使操作者可自由移动），Follower 单向跟随 Leader（`follower.reference = leader.response`，不反向）。

---

#### 5.3.2 `xarm_teleop_create_bilateral` — 创建双边力反馈控制器

```c
int xarm_teleop_create_bilateral(
    const char*             can_leader,
    const char*             can_follower,
    const char*             leader_urdf_path,
    const char*             follower_urdf_path,
    const char*             arm_side,
    xarm_teleop_handle_t*   out
);
```

参数与 `create_unilateral` 完全相同，区别仅在控制逻辑（双向耦合，含力反馈）。当前不属于 `publish` 主线默认交付入口。

---

#### 5.3.3 `xarm_teleop_create_gravity_comp` — 创建重力补偿控制器

```c
int xarm_teleop_create_gravity_comp(
    const char*             can_if,     // CAN 接口，如 "can0"
    const char*             urdf_path,  // 机械臂 URDF 文件绝对路径
    const char*             arm_side,   // "left_arm" 或 "right_arm"
    xarm_teleop_handle_t*   out
);
```

仅对单臂施加重力补偿，使操作者可自由拖动机械臂（零力感）。常用于示教模式数据采集。

---

#### 5.3.4 `xarm_teleop_start` — 启动控制循环

```c
int xarm_teleop_start(xarm_teleop_handle_t h);
```

非阻塞，启动内部三线程（Leader、Follower、Admin，均 500Hz）后立即返回。`create` 成功后必须显式调用 `start`，否则不会开始控制。

---

#### 5.3.5 `xarm_teleop_stop` — 请求停止

```c
int xarm_teleop_stop(xarm_teleop_handle_t h);
```

设置停止标志，控制线程在当前周期结束后安全退出。可在信号处理器中调用（非阻塞）。

---

#### 5.3.6 `xarm_teleop_wait` — 等待停止完成

```c
int xarm_teleop_wait(xarm_teleop_handle_t h);
```

阻塞等待所有内部线程退出，通常在 `stop` 后调用。若需同时处理信号或 ROS2 事件，改用 `is_running` 轮询。

---

#### 5.3.7 `xarm_teleop_is_running` — 查询运行状态

```c
int xarm_teleop_is_running(xarm_teleop_handle_t h);
```

返回值：`1` = 运行中，`0` = 已停止，`< 0` = 句柄无效。

---

#### 5.3.8 `xarm_teleop_destroy` — 销毁控制器

```c
void xarm_teleop_destroy(xarm_teleop_handle_t h);
```

释放所有资源。**销毁前必须先调用 `stop` 和 `wait`**，否则内部线程可能仍在运行。

---

#### 5.3.9 `xarm_teleop_set_joint_state_callback` — 注册关节状态回调

```c
typedef void (*xarm_teleop_joint_state_cb_t)(
    const float* positions,   // 7 个关节位置（rad）
    const float* velocities,  // 7 个关节速度（rad/s）
    const float* torques,     // 7 个关节力矩（Nm）
    int          count,       // 关节数量（通常为 7）
    void*        user_data    // 用户自定义数据
);

int xarm_teleop_set_joint_state_callback(
    xarm_teleop_handle_t       h,
    xarm_teleop_joint_state_cb_t cb,
    void*                      user_data
);
```

每个控制周期（约 2ms）触发一次，传入 Leader 最新关节状态。用于 ROS2 集成时发布 `/joint_states` 话题，或用于数据采集/可视化。

---

#### 5.3.10 `xarm_teleop_go_home` — 执行回零

```c
int xarm_teleop_go_home(xarm_teleop_handle_t h);
```

控制机械臂（Follower）回到零点位置。内部执行 AdjustPosition 插值运动，回零期间**阻塞**，完成后返回。

---

#### 5.3.11 `xarm_teleop_get_last_error` / `xarm_teleop_version`

```c
int xarm_teleop_get_last_error(char* buffer, int buffer_len);
const char* xarm_teleop_version(void);
```

`get_last_error` 返回最近一次 API 调用的错误描述字符串，用于调试和日志。`version` 返回版本字符串，如 `"1.0.0"`。

---

### 5.4 典型调用时序

```text
create_* → start → [is_running / callback / go_home] → stop → wait → destroy
```

---

### 5.5 信号处理（推荐模板）

```c
#include <signal.h>
#include "xarm_teleop_sdk.h"

static volatile sig_atomic_t g_stop_requested = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop_requested = 1;  // 信号处理器中只做最简单的标志位设置
}

int main(void) {
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    xarm_teleop_handle_t h = NULL;
    xarm_teleop_create_unilateral("can0", "can2", leader_urdf, follower_urdf, "right_arm", &h);
    xarm_teleop_start(h);

    // 主线程轮询，避免在信号处理器中执行复杂操作
    while (!g_stop_requested && xarm_teleop_is_running(h) == 1) {
        // 处理其他事件
    }

    xarm_teleop_stop(h);
    xarm_teleop_wait(h);
    xarm_teleop_destroy(h);
    return 0;
}
```

> **注意**：不要在信号处理函数中直接调用 `stop`/`destroy`，否则可能在 Ctrl+C 时卡死。主线程轮询是更安全的模式。

---

## 5.6 常见问题排查

### 5.6.1 程序启动后电机不响应

| 检查项 | 命令 |
|--------|------|
| CAN 接口是否存在 | `ip link show \| grep can` |
| CAN 接口是否 UP | `sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 fd on` |
| URDF 路径是否正确 | 查看 `xarm_teleop_get_last_error()` 输出 |
| 是否已调用 `start` | `create` 成功后必须显式调用 `start` |

### 5.6.2 编译失败：找不到 `libxarm_teleop_lib.so`

```bash
# 检查预编译库是否存在
ls publish/modules/src/xarm_teleop/prebuilt/xarm_teleop/lib/

# 检查 XARM_SDK_ROOT 是否正确
echo $XARM_SDK_ROOT
# 应输出：/path/to/publish/xarm_can/package
```

### 5.6.3 程序退出后电机仍在使能状态

原因：`stop`/`destroy` 未被调用，或在信号处理函数中执行了复杂操作导致卡死。

解决：确保主线程退出路径执行完整的退出模板（见 5.5 节）。
