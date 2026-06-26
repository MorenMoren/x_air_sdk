# xarm_can SDK 使用指南

---

## 目录

1. [SDK 目录结构](#1-sdk-目录结构)
2. [接口总览](#2-接口总览)
3. [API 详细说明](#3-api-详细说明)
4. [快速上手](#4-快速上手)
5. [两个 Demo 详细讲解](#5-两个-demo-详细讲解)
6. [典型调用时序](#6-典型调用时序)
7. [常见问题排查指南](#7-常见问题排查指南)

---

## 1. SDK 目录结构

```text
publish/xarm_can/package/
├── include/
│   ├── xarm_can_sdk.h        # 对外唯一头文件（C ABI，稳定接口契约）
│   └── xarm_sdk_export.h     # 符号导出宏
├── lib/
│   ├── libxarm_can_sdk.so    # 动态库文件
│   └── libxarm_can_sdk.so.1  # SONAME 软链接
├── examples/
│   ├── sdk_cpp_basic_test.cpp    # C++ 完整示例
│   └── sdk_python_basic_test.py  # Python 完整示例
└── libexec/
    ├── configure_socketcan.sh    # 配置单路 CAN 接口
    ├── setup_can_interfaces.sh   # 批量配置 CAN 接口（推荐）
    ├── set_zero.sh               # 电机零点设置
    └── change_baudrate.py        # 修改波特率工具
```

- `include`：对外头文件（稳定 C ABI，二次开发只需关注此目录）
- `lib`：动态库文件（`libxarm_can_sdk.so.1` 软链接必须存在，否则运行时报 `not found`）
- `examples`：最小可运行示例（C++ / Python）
- `libexec`：辅助脚本，用于 CAN 接口配置与电机初始化

---

## 2. 接口总览

> 头文件路径：`publish/xarm_can/package/include/xarm_can_sdk.h`

### 2.1 句柄与错误码

| 类型 / 常量 | 说明 |
|-------------|------|
| `xarm_sdk_handle_t` | SDK 会话句柄（不透明指针，不可直接解引用） |
| `XARM_SDK_OK` | 成功（值为 0） |
| `XARM_SDK_ERR_INVALID_ARGUMENT` | 参数错误（如空指针、ID 数量不符）|
| `XARM_SDK_ERR_INVALID_STATE` | 状态错误（如未初始化即调用控制接口） |
| `XARM_SDK_ERR_EXCEPTION` | 内部 C++ 异常（查看 `get_last_error` 了解详情） |

### 2.2 生命周期接口

| 接口 | 作用 |
|------|------|
| `xarm_sdk_get_version()` | 返回版本字符串，用于验证库加载正确 |
| `xarm_sdk_create(can_if, enable_fd, &out)` | 创建 SDK 会话句柄 |
| `xarm_sdk_destroy(handle)` | 销毁句柄并释放所有资源 |

### 2.3 初始化接口

| 接口 | 作用 |
|------|------|
| `xarm_sdk_init_arm_motors(h, types, send_ids, recv_ids, count)` | 初始化 7 个关节电机映射 |
| `xarm_sdk_init_gripper_motor(h, type, send_id, recv_id)` | 初始化 1 个夹爪电机映射 |

### 2.4 控制与通信接口

| 接口 | 作用 |
|------|------|
| `xarm_sdk_set_callback_mode_state_all(h)` | 设置全轴回调为状态上报模式（必须在使能前调用）|
| `xarm_sdk_enable_all(h)` | 全轴使能（含关节 + 夹爪） |
| `xarm_sdk_disable_all(h)` | 全轴失能（安全退出前必须调用）|
| `xarm_sdk_set_zero_all(h)` | 全轴置零（设置当前位置为零点，谨慎使用）|
| `xarm_sdk_refresh_all(h)` | 向所有电机发送状态查询请求 |
| `xarm_sdk_recv_all(h, timeout_us)` | 接收所有待处理的 CAN 反馈帧（解析后更新内部缓存）|
| `xarm_sdk_get_arm_joint_states(h, states, count)` | 读取 7 个关节电机当前状态 |
| `xarm_sdk_get_gripper_state(h, &state)` | 读取夹爪电机当前状态 |
| `xarm_sdk_arm_mit_control(h, params, count)` | 向关节电机发送 MIT 控制指令 |
| `xarm_sdk_gripper_mit_control(h, &param)` | 向夹爪电机发送 MIT 控制指令 |
| `xarm_sdk_gripper_open(h, kp, kd)` | 夹爪张开（快捷接口，内部封装 MIT 命令）|
| `xarm_sdk_gripper_close(h, kp, kd)` | 夹爪合拢（快捷接口）|

### 2.5 错误诊断接口

| 接口 | 作用 |
|------|------|
| `xarm_sdk_get_last_error(buf, len)` | 将最近一次错误信息写入 `buf`，用于诊断失败原因 |

---

## 3. API 详细说明

### 3.1 `xarm_sdk_create` — 创建 SDK 会话

```c
int xarm_sdk_create(const char *can_if, int enable_fd, xarm_sdk_handle_t *out);
```

**参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `can_if` | `const char*` | CAN 接口名，例如 `"can0"`，必须是系统中已存在且处于 UP 状态的接口 |
| `enable_fd` | `int` | 是否启用 CAN FD 模式：`0` = 经典 CAN，`1` = CAN FD。**必须与硬件配置一致** |
| `out` | `xarm_sdk_handle_t*` | 输出参数，成功时写入有效句柄，失败时写入 NULL |

**返回值：** `XARM_SDK_OK`（0）表示成功，非零表示失败。失败时调用 `xarm_sdk_get_last_error` 获取详细信息。

**示例（C++）：**
```cpp
xarm_sdk_handle_t h = nullptr;
int ret = xarm_sdk_create("can0", 1, &h);
if (ret != XARM_SDK_OK) {
    char err[256];
    xarm_sdk_get_last_error(err, sizeof(err));
    fprintf(stderr, "create failed: %s\n", err);
}
```

---

### 3.2 `xarm_sdk_init_arm_motors` — 初始化关节电机

```c
int xarm_sdk_init_arm_motors(
    xarm_sdk_handle_t h,
    const int *motor_types,
    const int *send_ids,
    const int *recv_ids,
    int count);
```

**参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `h` | 句柄 | 已通过 `xarm_sdk_create` 创建的有效句柄 |
| `motor_types` | `const int*` | 电机类型数组，长度为 `count`，使用 `XARM_SDK_MOTOR_*` 常量 |
| `send_ids` | `const int*` | 发送 CAN ID 数组，即主机向电机发命令时使用的 ID |
| `recv_ids` | `const int*` | 接收 CAN ID 数组，即电机回报状态时使用的 ID |
| `count` | `int` | 电机数量，当前 xarm 为 **7**（7 自由度关节）|

**电机类型常量（来自 `xarm_can_sdk.h`）：**

| 常量 | 说明 |
|------|------|
| `XARM_SDK_MOTOR_DM4310` | 达妙 DM4310 系列 |
| `XARM_SDK_MOTOR_DM4340` | 达妙 DM4340 系列 |
| `XARM_SDK_MOTOR_DM8009` | 达妙 DM8009 系列（夹爪常用）|

**send_id 与 recv_id 的关系：** 通常 `recv_id = send_id + 0x10`。例如关节 1 的 `send_id=0x01`，则 `recv_id=0x11`。但必须以实际硬件配置为准，不要假设。

**示例：**
```cpp
// 7 个关节的 ID 配置示例（以实际硬件为准）
int motor_types[7] = {
    XARM_SDK_MOTOR_DM4310, XARM_SDK_MOTOR_DM4310,
    XARM_SDK_MOTOR_DM4310, XARM_SDK_MOTOR_DM4310,
    XARM_SDK_MOTOR_DM4340, XARM_SDK_MOTOR_DM4340,
    XARM_SDK_MOTOR_DM4310
};
int send_ids[7] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int recv_ids[7] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
xarm_sdk_init_arm_motors(h, motor_types, send_ids, recv_ids, 7);
```

---

### 3.3 `xarm_sdk_init_gripper_motor` — 初始化夹爪电机

```c
int xarm_sdk_init_gripper_motor(
    xarm_sdk_handle_t h,
    int motor_type,
    int send_id,
    int recv_id);
```

**参数与 `init_arm_motors` 类似，但只针对单个夹爪电机。** 典型配置：
```cpp
xarm_sdk_init_gripper_motor(h, XARM_SDK_MOTOR_DM8009, 0x08, 0x18);
```

---

### 3.4 `xarm_sdk_recv_all` — 接收反馈帧

```c
int xarm_sdk_recv_all(xarm_sdk_handle_t h, int timeout_us);
```

**参数：**

| 参数 | 说明 |
|------|------|
| `timeout_us` | 等待数据的超时时间（微秒）。建议值：`500`（0.5ms）。过大会降低控制频率，过小可能漏读帧 |

**调用时机：** 每个控制周期中，先 `refresh_all()`（发出查询请求），再 `recv_all()`（读取响应）。然后才能读取最新状态。

---

### 3.5 `xarm_sdk_arm_mit_control` — 关节 MIT 控制

```c
int xarm_sdk_arm_mit_control(
    xarm_sdk_handle_t h,
    const xarm_sdk_mit_param_t *params,
    int count);
```

**参数：**

| 参数 | 说明 |
|------|------|
| `params` | MIT 参数数组，每个元素对应一个关节 |
| `count` | 关节数量（通常为 7）|

**`xarm_sdk_mit_param_t` 结构体：**

```c
typedef struct {
    float pos;     // 目标位置（rad）
    float vel;     // 目标速度（rad/s），通常设为 0.0
    float kp;      // 位置增益，建议范围 2.0 ~ 240.0
    float kd;      // 速度增益（阻尼），建议范围 0.2 ~ 8.0
    float torque;  // 前馈力矩（Nm），通常设为 0.0
} xarm_sdk_mit_param_t;
```

**控制示例（让关节 1 运动到 0.5 rad）：**
```cpp
xarm_sdk_mit_param_t params[7] = {};
// 只控制关节 1，其余保持当前位置（kp=0 时相当于力矩为 0）
params[0].pos = 0.5f;
params[0].kp  = 8.0f;
params[0].kd  = 0.8f;
xarm_sdk_arm_mit_control(h, params, 7);
```

---

### 3.6 `xarm_sdk_get_arm_joint_states` — 读取关节状态

```c
int xarm_sdk_get_arm_joint_states(
    xarm_sdk_handle_t h,
    xarm_sdk_joint_state_t *states,
    int count);
```

**`xarm_sdk_joint_state_t` 结构体：**

```c
typedef struct {
    float pos;     // 当前位置（rad）
    float vel;     // 当前速度（rad/s）
    float torque;  // 当前力矩估计值（Nm）
} xarm_sdk_joint_state_t;
```

**使用示例：**
```cpp
xarm_sdk_joint_state_t states[7];
xarm_sdk_refresh_all(h);
xarm_sdk_recv_all(h, 500);
xarm_sdk_get_arm_joint_states(h, states, 7);
for (int i = 0; i < 7; ++i) {
    printf("joint%d: pos=%.3f vel=%.3f torque=%.3f\n",
           i+1, states[i].pos, states[i].vel, states[i].torque);
}
```

---

### 3.7 `xarm_sdk_gripper_open` / `xarm_sdk_gripper_close` — 夹爪开合

```c
int xarm_sdk_gripper_open(xarm_sdk_handle_t h, float kp, float kd);
int xarm_sdk_gripper_close(xarm_sdk_handle_t h, float kp, float kd);
```

快捷接口，内部自动设置目标位置（open 对应 -1.0472 rad，close 对应 0.0 rad），只需传入增益：

```cpp
xarm_sdk_gripper_open(h, 5.0f, 0.5f);   // 张开夹爪
xarm_sdk_gripper_close(h, 5.0f, 0.5f);  // 合拢夹爪
```

---

### 3.8 安全退出模板

```cpp
// 标准安全退出流程
xarm_sdk_disable_all(h);
xarm_sdk_recv_all(h, 1000);  // 等待 1ms，确保失能命令已传输
xarm_sdk_destroy(h);
h = nullptr;
```

---

## 4. 快速上手

### 4.1 Step 1：配置 CAN 接口

```bash
cd /path/to/publish/xarm_can/package/libexec
bash setup_can_interfaces.sh

# 验证接口已 UP
ip link show | grep can
# 预期输出：can0: <NOARP,UP,LOWER_UP,ECHO> ...
```

若接口未出现，检查 CAN 适配器驱动和 USB 连接。

### 4.2 Step 2：编译 C++ 示例

```bash
g++ -std=c++17 \
  publish/xarm_can/package/examples/sdk_cpp_basic_test.cpp \
  -I publish/xarm_can/package/include \
  -L publish/xarm_can/package/lib \
  -Wl,-rpath,publish/xarm_can/package/lib \
  -lxarm_can_sdk \
  -o /tmp/sdk_cpp_basic_test

# 验证依赖库已找到
ldd /tmp/sdk_cpp_basic_test | grep xarm
# 应输出：libxarm_can_sdk.so.1 => /path/to/libxarm_can_sdk.so
```

### 4.3 Step 3：运行 C++ 示例

```bash
# 参数顺序：接口名  CAN-FD模式  演示时长(秒)  正弦幅值(rad)
/tmp/sdk_cpp_basic_test can0 1 12 0.20
```

| 参数 | 含义 | 典型值 |
|------|------|--------|
| 第 1 个 | CAN 接口名 | `can0` |
| 第 2 个 | CAN FD 模式（0=经典 CAN，1=CAN FD）| `1` |
| 第 3 个 | 演示时长（秒）| `12` |
| 第 4 个 | 正弦运动幅值（rad），约 11.5° | `0.20` |

### 4.4 Step 4：运行 Python 示例

```bash
python3 publish/xarm_can/package/examples/sdk_python_basic_test.py \
  --sdk publish/xarm_can/package/lib/libxarm_can_sdk.so \
  --can can0 \
  --fd  \
  --seconds 12 \
  --amp 0.20
```

---

## 5. 两个 Demo 详细讲解

### 5.1 C++ Demo：`sdk_cpp_basic_test.cpp`

执行流程（分阶段）：

| 阶段 | 操作 | 说明 |
|------|------|------|
| 1 | `create` | 创建 SDK 会话，连接 CAN 接口 |
| 2 | `init_arm_motors` + `init_gripper_motor` | 初始化 7 个关节 + 1 个夹爪电机 |
| 3 | `set_callback_mode_state_all` | 切换为状态上报模式 |
| 4 | `enable_all` | 使能所有电机，保持 2 秒观察 |
| 5 | `refresh + recv` × 10 | 验证通信链路，打印关节状态 |
| 6 | 正弦 MIT 控制循环 | 持续下发慢速正弦指令，打印实时位置 |
| 7 | 夹爪 MIT 演示 | 夹爪正弦振荡 3 秒 |
| 8 | 夹爪开合演示 | open → close 顺序操作 |
| 9 | 回零 | 所有关节回到 0 rad |
| 10 | `disable_all` + `recv` + `destroy` | 安全退出 |

### 5.2 Python Demo：`sdk_python_basic_test.py`

执行流程与 C++ Demo 完全对齐，使用 `ctypes` 绑定 SDK。

**关键概念：`argtypes` 和 `restype`**

Python 通过 `ctypes` 调用 C 函数时，必须显式声明参数类型和返回值类型，否则 Python 会按自己的规则传参，导致 C 函数收到错误数据：

```python
lib.xarm_sdk_arm_mit_control.argtypes = [
    ctypes.c_void_p,                       # 句柄
    ctypes.POINTER(XarmSdkMitParam),        # 参数数组指针
    ctypes.c_int,                           # 数量
]
lib.xarm_sdk_arm_mit_control.restype = ctypes.c_int
```

---

## 6. 典型调用时序

```
create
  └─→ init_arm_motors
  └─→ init_gripper_motor
       └─→ set_callback_mode_state_all
            └─→ enable_all
                 └─→ [ 控制循环 ]
                      │  refresh_all()
                      │  recv_all(h, 500)
                      │  get_arm_joint_states()  ← 读当前状态
                      │  arm_mit_control()        ← 发控制指令
                      └─→ [ 安全退出 ]
                           └─→ disable_all()
                           └─→ recv_all(h, 1000)
                           └─→ destroy()
```

---

## 7. 常见问题排查指南

### 7.1 `xarm_sdk_create` 失败

| 检查项 | 解决方法 |
|--------|---------|
| CAN 接口是否存在 | `ip link show \| grep can` |
| 接口是否 UP | `sudo ip link set can0 up type can bitrate 1000000` |
| CAN FD 模式是否匹配 | 查看硬件配置，`0`=经典 CAN，`1`=CAN FD |
| 权限是否足够 | `sudo` 运行，或将用户加入 `netdev` 组 |

### 7.2 编译通过但运行时找不到 `.so`

```bash
# 方法 1：设置 LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/path/to/publish/xarm_can/package/lib:$LD_LIBRARY_PATH

# 方法 2：编译时写入 RPATH（推荐）
g++ ... -Wl,-rpath,/absolute/path/to/lib ...

# 检查软链接是否存在（两个文件都必须存在）
ls -la /path/to/publish/xarm_can/package/lib/
```

### 7.3 程序运行但电机不动

| 检查项 | 说明 |
|--------|------|
| `motor_types` 是否正确 | 对照硬件型号查头文件常量 |
| `send_ids/recv_ids` 是否与现场一致 | 最常见问题，ID 不对则发出去没人应 |
| CAN FD 模式是否匹配 | 模式不匹配则帧格式不被识别 |
| 是否调用了 `enable_all` | 未使能则电机不响应控制指令 |

### 7.4 状态读取全为 0

原因：`recv_all` 未调用，或 `recv_ids` 配置错误导致状态帧无法被正确路由。

解决：
1. 确认调用顺序：`refresh_all()` → `recv_all(h, 500)` → `get_arm_joint_states()`
2. 确认 `recv_ids` 与硬件电机的回报 ID 完全一致
3. 使用 `candump can0` 实时抓包，确认电机确实有数据回报

### 7.5 退出时电机仍处于使能状态

原因：`disable_all` 之后立即关闭了 socket，命令可能未完成传输。

解决：
```cpp
xarm_sdk_disable_all(h);
xarm_sdk_recv_all(h, 1000);  // 等待 1ms，确保失能命令已发出
xarm_sdk_destroy(h);
```

### 7.6 缺少 `.so.1` 软链接

如果运行时报 `libxarm_can_sdk.so.1 not found`，手动创建软链接：
```bash
cd /path/to/publish/xarm_can/package/lib
ln -sf libxarm_can_sdk.so libxarm_can_sdk.so.1
```

---
