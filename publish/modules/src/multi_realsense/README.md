# multi_realsense

ROS 2 package for managing multiple Intel RealSense cameras simultaneously.

## 概述

该包提供了在 ROS 2 环境中同时管理多个 Intel RealSense 相机的功能，适用于需要多视角图像采集的机器人应用场景。

## 功能特性

- 支持同时运行多个 RealSense 相机（D435, D405等型号）
- 自动检测连接的相机序列号
- 灵活的启动配置（多相机/单相机模式）
- 可配置的图像分辨率和帧率
- 提供相机检查工具

## 依赖项

- ROS 2 Humble (或更高版本)
- `realsense2_camera` - Intel RealSense ROS 2 包
- `librealsense2` - Intel RealSense SDK

## 安装

### 1. 安装 RealSense SDK

```bash
sudo apt install ros-humble-realsense2-camera
```

### 2. 构建包

```bash
cd ~/your_workspace
colcon build --packages-select multi_realsense
source install/setup.bash
```

## 使用方法

### 检查连接的相机

在启动相机之前，建议先检查连接的相机：

```bash
ros2 run multi_realsense check_cameras.py
```

该命令会显示所有连接的 RealSense 相机的序列号和型号信息，并提供启动命令示例。

### 启动多个相机

根据检查到的序列号，启动多个相机：

```bash
ros2 launch multi_realsense multi_cameras.launch.py \
  serial_chest:=<胸部相机序列号> \
  serial_left:=<左手腕相机序列号> \
  serial_right:=<右手腕相机序列号>
```

例如：

```bash
ros2 launch multi_realsense multi_cameras.launch.py \
  serial_chest:=_314422070707 \
  serial_left:=_412622270856 \
  serial_right:=_230322273759
```

### 启动单个相机（测试用）

```bash
ros2 launch multi_realsense single_camera.launch.py \
  camera_name:=cam \
  serial_no:=_123456789
```

### Launch 参数

#### multi_cameras.launch.py

- `serial_chest`: 胸部相机序列号（默认：空）
- `serial_left`: 左手腕相机序列号（默认：空）
- `serial_right`: 右手腕相机序列号（默认：空）
- `auto_detect`: 自动检测相机序列号（默认：false）

#### single_camera.launch.py

- `camera_name`: 相机名称（默认：camera）
- `serial_no`: 相机序列号（默认：空）
- `color_width`: RGB 图像宽度（默认：640）
- `color_height`: RGB 图像高度（默认：480）
- `fps`: 帧率（默认：30）

## 话题

每个相机会在其命名空间下发布以下话题：

- `/<camera_name>/color/image_raw` - RGB 图像
- `/<camera_name>/color/camera_info` - 相机标定信息
- `/<camera_name>/depth/image_rect_raw` - 深度图像（如果启用）
- `/<camera_name>/aligned_depth_to_color/image_raw` - 对齐到彩色的深度图像（如果启用）


## 故障排除

### 相机无法检测

1. 确认相机连接到 USB 3.0 端口
2. 检查 USB 线缆是否正常
3. 验证 librealsense2 是否正确安装：
   ```bash
   rs-enumerate-devices
   ```

### 多相机 USB 带宽问题

运行多个相机可能会占用大量 USB 带宽，建议：
- 将相机连接到不同的 USB 控制器
- 降低分辨率或帧率
- 禁用不需要的数据流（如深度图）


## 许可证

本项目采用 Apache-2.0 许可证。详见 [LICENSE](LICENSE) 文件。

## 维护者

- vlai <<shiyu@vlai.cn>>

## 贡献

欢迎提交 Issue 和 Pull Request！
