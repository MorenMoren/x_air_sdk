# ROS TCP Endpoint (ROS2 版)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 简介

**ROS TCP Endpoint** 是 Unity 官方提供的 ROS2 包，作为 Unity 场景与 ROS2 网络之间的通信桥接层。它通过 TCP Socket 接收来自 Unity 端（[ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)）的消息，并将其转发到 ROS2 网络；同时也能将 ROS2 话题消息推送给 Unity。

该包是 [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) 集成方案的核心组件之一，适用于：

- 🤖 机器人仿真可视化（Unity 渲染 + ROS 逻辑）
- 🕶️ VR/AR 遥操作界面
- 📊 机器人数据实时可视化
- 🧪 算法开发与调试

---

## 工作原理

```
┌──────────────┐     TCP Socket      ┌──────────────────┐
│   Unity 端   │ ◄──────────────────► │  ROS TCP Endpoint │
│  (C# 脚本)   │      port 10000      │   (ROS2 节点)     │
└──────────────┘                      └────────┬─────────┘
                                               │
                                               ▼
                                        ┌──────────────────┐
                                        │    ROS2 网络      │
                                        │  (话题 / 服务)    │
                                        └──────────────────┘
```

1. Unity 端的 **ROS TCP Connector** (C#) 通过 TCP 连接到此 Endpoint 服务
2. Unity 可以发布话题消息 → Endpoint 收到后以 ROS2 话题发布出去
3. Unity 可以订阅 ROS2 话题 → Endpoint 收到 ROS2 消息后通过 TCP 发送给 Unity
4. 支持 ROS2 Service 的双向调用（Unity 调用 ROS Service / ROS 调用 Unity Service）

---

## 系统要求

- **ROS2 发行版**：Humble / Foxy / Galactic 等（支持 rclpy 的版本）
- **Python**：3.8+
- **Unity**：2020.3+（配合 [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) 使用）

---

## 安装

### 1. 克隆到 ROS2 工作区

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

### 2. 安装依赖

```bash
pip install -r ROS-TCP-Endpoint/requirements.txt
```

### 3. 编译

```bash
cd ~/your_ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

---

## 快速使用

### 启动默认 Endpoint 服务

```bash
ros2 launch ros_tcp_endpoint endpoint.py
```

默认参数：
- IP：`0.0.0.0`（监听所有网络接口）
- 端口：`10000`

### 自定义 IP 和端口

可通过 ROS2 参数覆盖默认配置：

```bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=192.168.1.100 -p ROS_TCP_PORT:=10001
```

或者修改 `launch/endpoint.py` 中的参数值。

### 验证服务是否启动

```bash
# 查看节点是否在运行
ros2 node list
# 应能看到 /UnityEndpoint

# 查看日志输出
ros2 node info /UnityEndpoint
```

---

## 编程接口

### 使用 `TcpServer` 类的完整示例

```python
import rclpy
from ros_tcp_endpoint import TcpServer

def main(args=None):
    rclpy.init(args=args)

    # 创建 TCP 服务器节点
    tcp_server = TcpServer("UnityEndpoint")

    # 可选：预注册话题和服务，避免 Unity 侧在运行时动态注册
    publishers = {
        # "topic_name": RosPublisher("topic_name", MsgType, queue_size)
    }
    subscribers = {
        # "topic_name": RosSubscriber("topic_name", MsgType, tcp_server, queue_size)
    }

    # 启动服务器（开始监听 TCP 连接）
    tcp_server.start(publishers, subscribers)

    # 进入事件循环（多线程执行器）
    tcp_server.setup_executor()

    # 清理
    tcp_server.destroy_nodes()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### 动态话题注册（从 Unity 端）

Unity 端可以通过系统命令（topic 以 `__` 开头）在运行时动态注册新的话题：

| 系统命令 | 功能 | Unity 调用示例 |
|---------|------|---------------|
| `__subscribe` | 订阅 ROS2 话题 | `__subscribe(topic, "std_msgs/String")` |
| `__publish` | 注册发布器 | `__publish(topic, "geometry_msgs/Twist")` |
| `__ros_service` | 调用 ROS2 服务 | `__ros_service(service_name, "std_srvs/Trigger")` |
| `__unity_service` | 注册 Unity 端实现的服务 | `__unity_service(service_name, "custom_srvs/Move")` |
| `__topic_list` | 获取当前所有话题列表 | `__topic_list()` |

> Unity 端的 ROS TCP Connector 包封装了这些调用，通常无需手动处理。

---

## 核心模块说明

| 模块 | 文件 | 说明 |
|------|------|------|
| `TcpServer` | `server.py` | 主服务器类，初始化 ROS 节点和 TCP Socket 监听 |
| `ClientThread` | `client.py` | 每个 Unity 连接对应一个线程，负责消息读取和分发 |
| `RosPublisher` | `publisher.py` | 将来自 Unity 的消息发布到 ROS2 话题 |
| `RosSubscriber` | `subscriber.py` | 订阅 ROS2 话题并将消息转发给 Unity |
| `RosService` | `service.py` | 接收 Unity 的请求并调用 ROS2 服务 |
| `UnityService` | `unity_service.py` | 注册 ROS2 服务，由 Unity 端实现服务逻辑 |
| `UnityTcpSender` | `tcp_sender.py` | 管理向 Unity 发送数据的线程和队列 |
| `ThreadPauser` | `thread_pauser.py` | 辅助工具，用于暂停线程等待 Unity 服务响应 |

### 通信协议格式

每条消息按以下二进制格式通过 TCP 传输：

```
┌─────────────────────────────────────────────┐
│ 4 bytes │ 目标名称长度（小端 int32）          │
├─────────────────────────────────────────────┤
│ N bytes │ 目标名称（UTF-8 字符串）            │
│         │ 例如："/cmd_vel" 或 "__subscribe"   │
├─────────────────────────────────────────────┤
│ 4 bytes │ 消息体长度（小端 int32）             │
├─────────────────────────────────────────────┤
│ M bytes │ 序列化后的 ROS2 消息内容            │
└─────────────────────────────────────────────┘
```

---

## 与 EmbodiedOS 的集成

在本仓库（EmbodiedOS）中，此 ROS TCP Endpoint 用于：

1. **Isaac Lab 仿真** → Unity 可视化渲染链路
2. **VR 遥操作**：Unity VR 端通过 TCP 发送控制指令 → Endpoint → ROS2 话题 → Isaac Lab 仿真
3. **数据回传**：仿真状态通过 ROS2 → Endpoint → Unity VR 端实时显示

典型启动流程：

```bash
# 终端 1：启动 ROS TCP Endpoint
ros2 launch ros_tcp_endpoint endpoint.py

# 终端 2：启动 Isaac Lab 仿真（参考 isaaclab 目录）
# 终端 3：启动 Unity 端 VR 应用
```

---

## 配置参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `ROS_IP` | string | `0.0.0.0` | TCP 服务器绑定的 IP 地址 |
| `ROS_TCP_PORT` | int | `10000` | TCP 服务器监听的端口号 |

可在以下位置配置：
- `launch/endpoint.py` 中的 `parameters` 字典
- 通过命令行 `--ros-args -p` 传入
- 代码中通过 `TcpServer` 构造函数的 `tcp_ip` 和 `tcp_port` 参数传入

---

## 项目结构

```
ROS-TCP-Endpoint-main-ros2/
├── launch/
│   └── endpoint.py              # ROS2 launch 启动文件
├── ros_tcp_endpoint/
│   ├── __init__.py              # 导出 TcpServer
│   ├── client.py                # 客户端连接线程（消息编解码）
│   ├── communication.py         # RosSender / RosReceiver 基类
│   ├── default_server_endpoint.py # 默认服务端入口脚本
│   ├── exceptions.py            # 自定义异常
│   ├── publisher.py             # ROS2 发布器封装
│   ├── server.py                # TCP 服务器主逻辑
│   ├── service.py               # ROS2 服务调用封装
│   ├── subscriber.py            # ROS2 订阅器封装
│   ├── tcp_sender.py            # Unity 消息发送队列管理
│   ├── thread_pauser.py         # 线程等待工具（用于服务调用）
│   └── unity_service.py         # Unity 端服务注册封装
├── resource/
│   └── ros_tcp_endpoint         # ament 资源索引
├── test/                        # 单元测试
├── package.xml                  # ROS2 包描述
├── setup.py                     # Python 包安装脚本
├── setup.cfg                    # 安装配置
├── requirements.txt             # Python 依赖
└── README.md                    # 英文原版文档
```

---

## 版本

当前版本：**v0.7.0**（ROS2 版）

协议版本通过 TCP 握手阶段从 Unity 端同步确认。

---

## 许可证

[Apache License 2.0](LICENSE)

本包由 Unity Technologies 开发并维护。  
更多信息请访问 [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)。

社区支持：[Unity Robotics 论坛](https://forum.unity.com/forums/robotics.623/)  
问题反馈：[GitHub Issues](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues)
