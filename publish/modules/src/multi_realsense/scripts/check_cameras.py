#!/usr/bin/env python3
# Copyright 2026 vlai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
检查连接的 RealSense 相机
"""

import subprocess
import sys


def check_cameras():
    """检查并显示连接的相机信息"""
    print("=" * 60)
    print("检查 RealSense 相机...")
    print("=" * 60)
    
    try:
        # 运行 rs-enumerate-devices
        result = subprocess.run(
            ['rs-enumerate-devices'],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode != 0:
            print("❌ 错误: 无法枚举设备")
            print(result.stderr)
            return False
        
        output = result.stdout

        # 解析输出
        devices = []
        current_device = {}
        
        for line in output.split('\n'):
            line = line.strip()
            
            if line.startswith('Device info:'):
                if current_device:
                    devices.append(current_device)
                current_device = {}
            
            elif 'Name' in line and ':' in line:
                current_device['name'] = line.split(':', 1)[1].strip()
            
            elif 'Serial Number' in line and ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                if 'Asic Serial Number' in key:
                    current_device['asic_serial'] = value
                elif 'Serial Number' in key:
                    current_device['serial'] = value
            
            elif 'Product Id' in line and ':' in line:
                current_device['product_id'] = line.split(':', 1)[1].strip()
        
        if current_device:
            devices.append(current_device)
        
        # 显示结果
        if not devices:
            print("❌ 未检测到 RealSense 相机")
            print("\n请检查:")
            print("  1. 相机是否连接到 USB 3.0 端口")
            print("  2. USB 线缆是否良好")
            print("  3. 是否安装了 librealsense2")
            return False
        
        print(f"\n✅ 检测到 {len(devices)} 个相机:\n")
        
        for i, device in enumerate(devices, 1):
            print(f"相机 {i}:")
            print(f"  名称: {device.get('name', 'Unknown')}")
            print(f"  序列号: {device.get('serial', 'Unknown')}")
            print(f"  产品 ID: {device.get('product_id', 'Unknown')}")
            print()
        
        # 生成启动命令
        print("=" * 60)
        print("启动命令示例:")
        print("=" * 60)
        
        if len(devices) >= 2:
            print(f"\n# 启动所有 {len(devices)} 个相机:")
            print(f"ros2 launch multi_realsense multi_cameras.launch.py \\")
            print(f"  serial_chest:=_{devices[0].get('serial', '')} \\")
            print(f"  serial_right:=_{devices[1].get('serial', '')} \\")
            print(f"  serial_left:=_{devices[2].get('serial', '')}")
        else:
            print(f"\n⚠️  警告: 只检测到 {len(devices)} 个相机，需要 3 个")
        
        print("\n# 测试单个相机:")
        if devices:
            print(f"ros2 launch multi_realsense single_camera.launch.py \\")
            print(f"  camera_name:=test_cam \\")
            print(f"  serial_no:=_{devices[0].get('serial', '')}")
        
        print("\n" + "=" * 60)
        
        return True
        
    except FileNotFoundError:
        print("❌ 错误: 找不到 rs-enumerate-devices")
        print("\n请安装 librealsense2:")
        print("  sudo apt install ros-humble-realsense2-camera")
        return False
    
    except Exception as e:
        print(f"❌ 错误: {e}")
        return False


if __name__ == '__main__':
    success = check_cameras()
    sys.exit(0 if success else 1)
