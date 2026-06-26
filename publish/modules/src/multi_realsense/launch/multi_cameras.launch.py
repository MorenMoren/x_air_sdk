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
多相机启动文件
支持 3 个 RealSense D435 相机
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import subprocess
import re


def get_realsense_serial_numbers():
    """自动检测连接的 RealSense 相机序列号"""
    try:
        result = subprocess.run(
            ['rs-enumerate-devices', '-s'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        # 解析序列号
        serial_numbers = []
        for line in result.stdout.split('\n'):
            if 'Serial Number' in line:
                match = re.search(r':\s*(\S+)', line)
                if match:
                    serial_numbers.append(match.group(1))
        
        return serial_numbers
    except Exception as e:
        print(f"警告: 无法自动检测相机序列号: {e}")
        return []


def launch_camera_node(context, *args, **kwargs):
    """动态生成相机节点"""
    
    # 获取启动参数
    use_auto_detect = LaunchConfiguration('auto_detect').perform(context)
    
    # 相机配置
    camera_configs = [
        {
            'name': 'cam_chest',
            'serial': LaunchConfiguration('serial_chest').perform(context),
            'device_type': 'd435',
            'color_width': 1280,
            'color_height': 720,
            'fps': 30,
        },
        {
            'name': 'cam_wrist_left',
            'serial': LaunchConfiguration('serial_left').perform(context),
            'device_type': 'd405',
            'color_width': 848,
            'color_height': 480,
            'fps': 30,
        },
        {
            'name': 'cam_wrist_right',
            'serial': LaunchConfiguration('serial_right').perform(context),
            'device_type': 'd405',
            'color_width': 848,
            'color_height': 480,
            'fps': 30,
        }
    ]
    
    # 如果启用自动检测，使用检测到的序列号
    if use_auto_detect == 'true':
        detected_serials = get_realsense_serial_numbers()
        print(f"检测到 {len(detected_serials)} 个相机: {detected_serials}")
        
        for i, config in enumerate(camera_configs):
            if i < len(detected_serials):
                config['serial'] = detected_serials[i]
                print(f"  {config['name']}: {config['serial']}")
    
    # 创建相机节点
    nodes = []
    for config in camera_configs:
        if not config['serial'] or config['serial'] == '':
            print(f"跳过 {config['name']} - 未指定序列号")
            continue
        
        # 相机参数
        parameters = [
            {
                'camera_name': config['name'],
                'serial_no': config['serial'],
                'device_type': config['device_type'],
                
                # RGB 配置
                'enable_color': True,
                'rgb_camera.profile': f"{config['color_width']}x{config['color_height']}x{config['fps']}",
                'rgb_camera.color_format': 'RGB8',
                
                # 禁用深度
                'enable_depth': False,
                
                # 禁用对齐
                'align_depth.enable': False,
                
                # 惯性测量单元 (禁用)
                'enable_gyro': False,
                'enable_accel': False,
                
                # 其他传感器 (禁用)
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_fisheye': False,
                
                # 性能优化
                'publish_tf': True,
                'tf_publish_rate': 0.0,  # 仅发布一次静态 TF

                # 图像传输设置 - 使用原始格式
                'image_transport': 'raw',
            }
        ]
        
        # 创建节点
        camera_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=config['name'],
            namespace=config['name'],
            parameters=parameters,
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
        )
        
        nodes.append(camera_node)
    
    return nodes


def generate_launch_description():
    """生成启动描述"""
    
    return LaunchDescription([
        # 设置环境变量 - 禁用压缩深度图像传输插件
        SetEnvironmentVariable(
            'ROS_IMAGE_TRANSPORT_PLUGINS',
            'image_transport/raw_pub:image_transport/raw_sub'
        ),
        
        # 启动参数
        DeclareLaunchArgument(
            'auto_detect',
            default_value='false',
            description='自动检测相机序列号'
        ),
        
        DeclareLaunchArgument(
            'serial_chest',
            default_value='',
            description='胸部相机序列号'
        ),
        
        DeclareLaunchArgument(
            'serial_left',
            default_value='',
            description='左手腕相机序列号'
        ),
        
        DeclareLaunchArgument(
            'serial_right',
            default_value='',
            description='右手腕相机序列号'
        ),
        
        # 动态生成相机节点
        OpaqueFunction(function=launch_camera_node),
    ])
