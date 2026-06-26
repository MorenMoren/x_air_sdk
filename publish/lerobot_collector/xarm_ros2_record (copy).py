#!/usr/bin/env python3
"""
XArm ROS2 数据采集脚本 - LeRobot v3.0 格式
missing episode 11 
使用示例:
    # 右臂数据采集
    python xarm_ros2_record.py \
        --repo-id myuser/xarm_dataset_right \
        --root ~/lerobot_datasets \
        --arm-side right_arm \
        --single-task "pick and place" \
        --num-episodes 50 \
        --use-wrist-camera
    
    # 左臂数据采集
    python xarm_ros2_record.py \
        --repo-id myuser/xarm_dataset_left \
        --num-episodes 50 \
        --use-depth-camera \
        --single-task "pick and place" \
        --root ~/lerobot_datasets_depth_nowrist_pick

ROS2 话题说明 (右臂示例):
    - /cam_chest/cam_chest/color/image_raw (观察相机 - 通用)
    - /cam_wrist_right/cam_wrist_right/color/image_rect_raw (右臂手腕相机)
    - /xarm_right_leader/arm/position (action - 关节位置)
    - /xarm_right_leader/hand/position (action.gripper - 夹爪)
    - /xarm_right_follower/arm/position (observation.state - 关节位置)
    - /xarm_right_follower/hand/position (observation.gripper_state - 夹爪)

ROS2 话题说明 (左臂示例):
    - /cam_chest/cam_chest/color/image_raw (观察相机 - 通用)
    - /cam_wrist_left/cam_wrist_left/color/image_rect_raw (左臂手腕相机)
    - /xarm_left_leader/arm/position (action - 关节位置)
    - /xarm_left_leader/hand/position (action.gripper - 夹爪)
    - /xarm_left_follower/arm/position (observation.state - 关节位置)
    - /xarm_left_follower/hand/position (observation.gripper_state - 夹爪)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Empty
from std_srvs.srv import Trigger
import numpy as np
import threading
import time
import argparse
from cv_bridge import CvBridge
from pathlib import Path
from typing import Dict, Optional
import logging

# LeRobot imports
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame
from lerobot.utils.constants import ACTION
from evdev import InputDevice,categorize,ecodes
from  xarm_trajectory_executor import *

class XArmROSCollector(Node):
    """XArm 机器人 ROS2 数据采集节点"""
    
    def __init__(
        self,
        repo_id: str,
        root: str,
        fps: int = 30,
        single_task: str = "default_task",
        num_episodes: int = 50,
        auto_home_timeout: float = 30.0,
        arm_side: str = "right_arm",
        use_wrist_camera: bool = False,
        use_depth_camera: bool = False,
    ):
        super().__init__('xarm_lerobot_collector')
        
        # 基本参数
        self.repo_id = repo_id
        self.root = Path(root).expanduser().resolve()
        self.fps = fps
        self.task = single_task
        self.num_episodes = num_episodes
        self.home_timeout = auto_home_timeout
        self.use_wrist_camera = use_wrist_camera
        self.use_depth_camera = use_depth_camera
        
        # 臂侧参数 - 转换为短名（right_arm -> right）
        self.arm_side = arm_side
        self.arm_side_short = "right" if arm_side == "right_arm" else "left"
        self.wrist_camera_key = f'observation.images.cam_wrist_{self.arm_side_short}'
        self.wrist_camera_key_list = ['observation.images.cam_wrist_left','observation.images.cam_wrist_right']
        self.get_logger().info(f"🎯 Arm side: {self.arm_side} (prefix: {self.arm_side_short})")
        self.get_logger().info(f"📷 Use wrist camera: {self.use_wrist_camera}")
        
        # 控制变量
        self.is_recording = False
        self.episode_buffer = []
        self.current_episode_idx = 0
        self.lock = threading.Lock()
        self.current_task_phase = 1
        # 当前帧数据缓存
        frame_keys = {
            'observation.images.cam_chest': None,
            'observation.state': None,  # 8维: 7关节 + 1夹爪
            'action': None,  # 8维: 7关节 + 1夹爪
            "observation.task_phase": np.array([1],dtype=np.int64),
        }
        
        # 如果使用手腕相机，添加到帧缓存
        if self.use_wrist_camera:
            for key in self.wrist_camera_key_list:
                frame_keys[key] = None
        if self.use_depth_camera:
            frame_keys['observation.depth'] = None
        
        self.current_frame = frame_keys
        self.bridge = CvBridge()
        # 临时存储分离的关节和夹爪数据（用于合并）
        self._temp_follower_joints = None  # 7维
        self._temp_follower_gripper = None  # 1维
        self._temp_leader_joints = None  # 7维
        self._temp_leader_gripper = None  # 1维
        
        self.right_temp_follower_joints = None  # 7维
        self.right_temp_follower_gripper = None  # 1维
        self.right_temp_leader_joints = None  # 7维
        self.right_temp_leader_gripper = None  # 1维
        # 时间戳记录（用于同步检查）
        self.frame_timestamps = {}
        
        # 初始化数据集
        self.dataset = self._create_lerobot_dataset()
        
        # 设置订阅
        self._setup_ros_subscriptions()
        
        # 复位服务客户端
        #self.home_client = self.create_client(Trigger, 'robot_go_home')
        #if self.home_client.wait_for_service(timeout_sec=3.0):
            #self.get_logger().info("✅ Robot home service connected")
        #else:
        #    self.get_logger().warn("⚠️  Robot home service not available")
        
        # 定时器
        self.timer = self.create_timer(1.0 / self.fps, self._collect_frame_callback)
        
        # 键盘监听
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        #self.mouse_keyboard_node = InputDevice('/dev/input/event13')
        #self.mouse_thread = threading.Thread(target=self._mouse_listener, daemon=True)
        #triggerself.mouse_thread.start()
        self.get_logger().info(f"📁 Dataset path: {self.dataset.root}")
        self.get_logger().info("⌨️  Controls:")
        self.get_logger().info("   'r' - Start/Stop recording episode")
        self.get_logger().info("   'h' - Go home (return to initial position)")
        self.get_logger().info("   'n' - Save episode and go home")
        self.get_logger().info("   'q' - Quit and save")

    def _create_lerobot_dataset(self) -> LeRobotDataset:
        """创建 LeRobot 数据集 (8维: 7关节+1夹爪)"""
        
        # 定义特征
        joint_count = 14
        gripper_dim = 2
        total_dim = joint_count + gripper_dim  # 8维
        
        # 定义数据集特征（LeRobot v3.0 格式）
        # ⭐ 夹爪合并到 state 和 action 的最后一维
        features = {
            # 图像观察 - 胸部相机（必需）
            'observation.images.cam_chest': {
                'dtype': 'video',
                'shape': (3, 720, 1280),
                'names': ['channel', 'height', 'width'],
            },
            "observation.task_phase":{
                'dtype': "int64",
                'shape': (1,),
                'names': ["phase_id"],
            }
        }
        
        # 如果使用深度相机，添加到特征中
        if self.use_depth_camera:
            features['observation.depth'] = {
                'dtype': 'uint16',
                'shape': (720, 1280),
                'names': ['height', 'width'],
            }
        
        # 如果使用手腕相机，添加到特征中
        if self.use_wrist_camera:
            for wrist_key in self.wrist_camera_key_list:
                features[wrist_key] = {
                'dtype': 'video',
                'shape': (3, 480, 848),
                'names': ['channel', 'height', 'width'],
            }
        
        # 添加状态和动作特征
        features.update({
            # 状态观察 - 8维 (7关节 + 1夹爪)
            'observation.state': {
                'dtype': 'float32',
                'shape': (total_dim,),
                'names': [f'left_joint_{i}' for i in range(7)] + ['gripper_position_left']+[f'right_joint_{i}' for i in range(7)] + ['gripper_position_right'],
            },
            # 动作 - 8维 (7关节 + 1夹爪)
            'action': {
                'dtype': 'float32',
                'shape': (total_dim,),
                'names': [f'left_joint_{i}' for i in range(7)] + ['gripper_position_left']+[f'right_joint_{i}' for i in range(7)] + ['gripper_position_right'],
            },
        })
        
        # 创建数据集
        dataset = LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            root=self.root,
            robot_type='xarm',
            features=features,
            use_videos=True,
            image_writer_threads=24,  # 2个相机 x 4线程/相机 x 2倍余量
        )
        
        self.get_logger().info(f"✅ Created LeRobot dataset: {self.repo_id}")
        return dataset

    def _setup_ros_subscriptions(self):
        """设置 ROS2 订阅"""
        # 相机订阅 - 胸部相机（必需）
        self.create_subscription(
            Image,
            '/cam_chest/cam_chest/color/image_raw',
            lambda msg: self._image_callback(msg, 'observation.images.cam_chest'),
            10
        )
        
        # 根据 arm_side 选择手腕相机（可选）
        if self.use_wrist_camera:
            wrist_left_camera_topic = '/cam_wrist_left/cam_wrist_left/color/image_raw'
            wrist_right_camera_topic = '/cam_wrist_right/cam_wrist_right/color/image_raw'
            self.create_subscription(
                Image,
                wrist_left_camera_topic,
                lambda msg: self._image_callback(msg, self.wrist_camera_key_list[0]),
                10
            )
            self.create_subscription(
                Image,
                wrist_right_camera_topic,
                lambda msg: self._image_callback(msg, self.wrist_camera_key_list[1]),
                10
            )
        if self.use_depth_camera:
            depth_camera_topic = '/cam_chest/cam_chest/aligned_depth_to_color/image_raw'
            self.create_subscription(
                Image,
                depth_camera_topic,
                lambda msg: self._depth_callback(msg, 'observation.depth'),
                10
            )
        # 根据 arm_side 生成话题前缀
        leader_prefix = f'/xarm_{self.arm_side_short}_leader'
        follower_prefix = f'/xarm_{self.arm_side_short}_follower'
        
        
        
        self.get_logger().info(f"📡 Subscribing to:")
        self.get_logger().info(f"   Chest camera:  /cam_chest/cam_chest/color/image_raw")
        if self.use_wrist_camera:
            self.get_logger().info(f"   Wrist camera:  {self.wrist_camera_key_list}")
        else:
            self.get_logger().info(f"   Wrist camera:  [Disabled]")
        self.get_logger().info(f"   Leader:   {leader_prefix}/*")
        self.get_logger().info(f"   Follower: {follower_prefix}/*")
        
        self.create_subscription(
            Float64MultiArray,
            f'/cmd_ctl_left',
            self._joycon_left_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/cmd_ctl_right',
            self._joycon_right_callback,
            10
        )

        # 关节和夹爪订阅
        self.create_subscription(
            Float64MultiArray,
            '/xarm_left_leader/arm/position',
            self._leader_joints_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_left_leader/hand/position',
            self._leader_gripper_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_left_follower/arm/position',
            self._follower_joints_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_left_follower/hand/position',
            self._follower_gripper_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_right_leader/arm/position',
            self.right_leader_joints_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_right_leader/hand/position',
            self.right_leader_gripper_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_right_follower/arm/position',
            self.right_follower_joints_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            f'/xarm_right_follower/hand/position',
            self.right_follower_gripper_callback,
            10
        )

    def _ros_image_to_numpy(self, msg: Image) -> np.ndarray:
        """将 ROS Image 消息转换为 numpy 数组 (H, W, C) RGB"""
        height, width = msg.height, msg.width
        encoding = msg.encoding
        
        # 确定数据类型
        dtype = np.uint8
        if encoding in ['16UC1', 'mono16']:
            dtype = np.uint16
        elif encoding == '32FC1':
            dtype = np.float32
        
        # 解析数据
        img_array = np.frombuffer(msg.data, dtype=dtype)
        
        # 根据编码格式重塑
        if encoding == 'rgb8':
            img = img_array.reshape((height, width, 3))
        elif encoding == 'bgr8':
            img = img_array.reshape((height, width, 3))[:, :, ::-1]  # BGR -> RGB
        elif encoding == 'rgba8':
            img = img_array.reshape((height, width, 4))[:, :, :3]
        elif encoding == 'bgra8':
            img = img_array.reshape((height, width, 4))[:, :, [2, 1, 0]]
        elif encoding == 'mono8':
            img = img_array.reshape((height, width, 1))
            img = np.repeat(img, 3, axis=2)
        elif encoding in ['mono16', '16UC1']:
            img = img_array.reshape((height, width, 1))
            img = (img / 256).astype(np.uint8)
            img = np.repeat(img, 3, axis=2)
        else:
            self.get_logger().error(f"Unsupported encoding: {encoding}")
            return np.zeros((height, width, 3), dtype=np.uint8)
        
        return img

    def _image_callback(self, msg: Image, key: str):
        """图像回调"""
        with self.lock:
            img = self._ros_image_to_numpy(msg)
            img = np.transpose(img, (2, 0, 1)) 
            self.current_frame[key] = img
            
            # 记录时间戳
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                self.frame_timestamps[key] = rclpy.time.Time.from_msg(msg.header.stamp)
            else:
                self.frame_timestamps[key] = self.get_clock().now()
    def _depth_callback(self, msg: Image, key: str):
        """深度图像回调"""
        with self.lock:
            try:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                self.current_frame[key] = depth
            
            except Exception as e:
                self.get_logger().error(f"Error converting depth image: {e}")

            # 记录时间戳
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                self.frame_timestamps[key] = rclpy.time.Time.from_msg(msg.header.stamp)
            else:
                self.frame_timestamps[key] = self.get_clock().now()
    def _joycon_left_callback(self, msg: Float64MultiArray):
        """Leader 关节位置 -> 临时存储，等待与夹爪合并"""
        # with self.lock:
        if not self.is_recording:
            self.get_logger().info("joycon trigger,start recording!!")
            self._start_recording()
        else:
            self.get_logger().info("joycon trigger,stop recording!!")
            self._stop_recording()
    
    def _joycon_right_callback(self, msg: Float64MultiArray):
        """Leader 关节位置 -> 临时存储，等待与夹爪合并"""
        # with self.lock:
        self.get_logger().info("joycon trigger, saving recording!!")
        if self.is_recording:
            self._stop_recording()
        self._save_episode()
    
    def _leader_joints_callback(self, msg: Float64MultiArray):
        """Leader 关节位置 -> 临时存储，等待与夹爪合并"""
        with self.lock:
            self._temp_leader_joints = np.array(msg.data, dtype=np.float32)
            self._merge_leader_data()

    def _leader_gripper_callback(self, msg: Float64MultiArray):
        """Leader 夹爪 -> 临时存储，等待与关节合并"""
        with self.lock:
            self._temp_leader_gripper = np.array(msg.data, dtype=np.float32)
            
            self._merge_leader_data()

    def _follower_joints_callback(self, msg: Float64MultiArray):
        """Follower 关节位置 -> 临时存储，等待与夹爪合并"""
        with self.lock:
            self._temp_follower_joints = np.array(msg.data, dtype=np.float32)
            self._merge_follower_data()

    def _follower_gripper_callback(self, msg: Float64MultiArray):
        """Follower 夹爪 -> 临时存储，等待与关节合并"""
        with self.lock:
            self._temp_follower_gripper = np.array(msg.data, dtype=np.float32)
            
            self._merge_follower_data()
    def right_leader_joints_callback(self, msg: Float64MultiArray):
        """Leader 关节位置 -> 临时存储，等待与夹爪合并"""
        with self.lock:
            self.right_temp_leader_joints = np.array(msg.data, dtype=np.float32)
            self._merge_leader_data()

    def right_leader_gripper_callback(self, msg: Float64MultiArray):
        """Leader 夹爪 -> 临时存储，等待与关节合并"""
        with self.lock:
            self.right_temp_leader_gripper = np.array(msg.data, dtype=np.float32)
            
            self._merge_leader_data()

    def right_follower_joints_callback(self, msg: Float64MultiArray):
        """Follower 关节位置 -> 临时存储，等待与夹爪合并"""
        with self.lock:
            self.right_temp_follower_joints = np.array(msg.data, dtype=np.float32)
            self._merge_follower_data()

    def right_follower_gripper_callback(self, msg: Float64MultiArray):
        """Follower 夹爪 -> 临时存储，等待与关节合并"""
        with self.lock:
            self.right_temp_follower_gripper = np.array(msg.data, dtype=np.float32)
            
            self._merge_follower_data()
            
    
    def _merge_leader_data(self):
        """合并 Leader 关节和夹爪数据到 action (8维)"""
        if self._temp_leader_joints is not None and self._temp_leader_gripper is not None and self.right_temp_leader_gripper is not None and self.right_temp_leader_joints is not None:
            # 合并: [14个关节] + [2个夹爪]
            self.current_frame['action'] = np.concatenate([
                self._temp_leader_joints,
                self._temp_leader_gripper,
                self.right_temp_leader_joints,
                self.right_temp_leader_gripper
            ])
            self.frame_timestamps['action'] = self.get_clock().now()
    
    def _merge_follower_data(self):
        """合并 Follower 关节和夹爪数据到 observation.state (8维)"""
        if self._temp_follower_joints is not None and self._temp_follower_gripper is not None and self.right_temp_follower_gripper is not None and self.right_temp_follower_joints is not None:
            # 合并: [14个关节] + [2个夹爪]
            self.current_frame['observation.state'] = np.concatenate([
                self._temp_follower_joints,
                self._temp_follower_gripper,
                self.right_temp_follower_joints,
                self.right_temp_follower_gripper
            ])
            self.frame_timestamps['observation.state'] = self.get_clock().now()

    def _collect_frame_callback(self):
        """定时收集帧数据"""
        if not self.is_recording:
            return
        
        with self.lock:
            # 检查所有数据是否就绪
            if any(v is None for v in self.current_frame.values()):
                missing = [k for k, v in self.current_frame.items() if v is None]
                self.get_logger().warn(
                    f'Missing data: {missing}',
                    throttle_duration_sec=2.0
                )
                return
            
            # 检查数据新鲜度（可选）
            current_time = self.get_clock().now()
            max_age_sec = 0.2
            stale_data = []
            
            for key, timestamp in self.frame_timestamps.items():
                if timestamp is not None:
                    age = (current_time - timestamp).nanoseconds / 1e9
                    if age > max_age_sec:
                        stale_data.append(f"{key}({age:.3f}s)")
            
            if stale_data:
                self.get_logger().warn(
                    f'Stale data: {stale_data}',
                    throttle_duration_sec=2.0
                )
            
            # 构建帧数据（使用 LeRobot 标准格式 - 夹爪已合并到 state 和 action）
            frame = {
                'observation.images.cam_chest': self.current_frame['observation.images.cam_chest'].copy(),
                'observation.state': self.current_frame['observation.state'].copy(),  # 8维
                'action': self.current_frame['action'].copy(),  # 8维
                "observation.task_phase":np.array([self.current_task_phase],dtype=np.int64)
            }
            
            # 如果使用手腕相机，添加到帧数据中
            if self.use_wrist_camera:
                for key in self.wrist_camera_key_list:
                    frame[key] = self.current_frame[key].copy() 
                # 'timestamp': current_time.nanoseconds / 1e9,
            if self.use_depth_camera:
                frame['observation.depth'] = self.current_frame['observation.depth'].copy()
            # ⭐ 关键: 添加帧到数据集 buffer
            try:
                frame["task"] = self.task
                self.dataset.add_frame(frame)
                
                # 获取当前 episode buffer 中的帧数（仅在 buffer 存在时）
                if "size" in self.dataset.episode_buffer:
                    current_frame_count = self.dataset.episode_buffer["size"]
                    
                    # 日志 - 使用 episode_buffer 的 size
                    if current_frame_count % 30 == 0:
                        self.get_logger().info(
                            f"Episode {self.current_episode_idx}, Frame {current_frame_count}"
                        )
            except Exception as e:
                self.get_logger().error(f"❌ Error adding frame: {str(e)}", throttle_duration_sec=1.0)
                # 如果 add_frame 失败，停止录制
                self.is_recording = False

    def _go_home(self) -> bool:
        """调用机器人复位服务"""
        try:
            self.get_logger().info("🏠 Calling robot go home...")
            dualarm_home()
            self.get_logger().info("🏠 robot go home success")
        except Exception as e:
            self.get_logger().error(f"❌ Error calling home: {str(e)}")
            return False
        # if not self.home_client.service_is_ready():
        #     self.get_logger().error("❌ Home service not available")
        #     return False
        
        # try:
        #     self.get_logger().info("🏠 Calling robot go home...")
        #     request = Trigger.Request()
        #     future = self.home_client.call_async(request)
            
        #     start_time = self.get_clock().now()
        #     timeout_duration = rclpy.duration.Duration(seconds=self.home_timeout)
            
        #     while rclpy.ok():
        #         rclpy.spin_once(self, timeout_sec=0.1)
                
        #         if future.done():
        #             break
                
        #         if (self.get_clock().now() - start_time) > timeout_duration:
        #             self.get_logger().error("❌ Go home timeout!")
        #             return False
            
        #     if future.done():
        #         response = future.result()
        #         if response.success:
        #             self.get_logger().info(f"✅ {response.message}")
        #             return True
        #         else:
        #             self.get_logger().error(f"❌ Home failed: {response.message}")
        #             return False
            
        #     return False
            
        # except Exception as e:
        #     self.get_logger().error(f"❌ Error calling home: {str(e)}")
        #     return False

    def _start_recording(self):
        """开始录制"""
        self.is_recording = True
        self.get_logger().info(f"📹 Recording episode {self.current_episode_idx} STARTED")

    def _stop_recording(self):
        """停止录制"""
        self.is_recording = False
        self.current_task_phase = 1
        self.get_logger().info("⏹️  Recording STOPPED")
    def _save_episode(self) -> bool:
        """保存 episode"""
        try:
            # 检查 episode_buffer 是否存在且有数据
            if not hasattr(self.dataset, 'episode_buffer') or not self.dataset.episode_buffer:
                self.get_logger().warn("No episode buffer to save")
                return False
            
            frame_count = self.dataset.episode_buffer.get("size", 0)
            
            if frame_count == 0:
                self.get_logger().warn("No frames to save")
                return False
            
            self.get_logger().info(
                f"💾 Saving episode {self.current_episode_idx} "
                f"with {frame_count} frames..."
            )
            
            # 调用 LeRobot 标准保存方法
            self.dataset.save_episode()
            
            self.get_logger().info(
                f"✅ Episode {self.current_episode_idx} saved successfully"
            )
            
            self.current_episode_idx += 1
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error saving episode: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    def _discard_episode(self) -> bool:
        """保存 episode"""
        try:
            # 检查 episode_buffer 是否存在且有数据
            if not hasattr(self.dataset, 'episode_buffer') or not self.dataset.episode_buffer:
                self.get_logger().warn("No episode buffer to discard")
                return False
            
            frame_count = self.dataset.episode_buffer.get("size", 0)
            
            self.get_logger().info(
                f"💾 discarding episode {self.current_episode_idx} "
                f"with {frame_count} frames..."
            )
            
            # 调用 LeRobot 标准保存方法
            self.dataset.clear_episode_buffer()
            
            self.get_logger().info(
                f"✅ Episode {self.current_episode_idx} cleared successfully"
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error discarding episode: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
            
    def _save_and_home(self) -> bool:
        """保存 episode 并复位"""
        # 先停止录制
        self.is_recording = False
        
        # 保存数据
        if not self._save_episode():
            return False
        
        # 复位机器人
        if self._go_home():
            self.get_logger().info("🆕 Ready for new episode")
            return True
        else:
            self.get_logger().warn("⚠️  Failed to go home, please reset manually")
            return False

    def _keyboard_listener(self):
        """键盘监听线程"""
        import sys
        import select
        
        try:
            while rclpy.ok():
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        line = sys.stdin.readline().strip().lower()
                        
                        if line == 'r':
                            if not self.is_recording:
                                self._start_recording()
                            else:
                                self._stop_recording()
                        
                        elif line == 's':
                            if self.is_recording:
                                self._stop_recording()
                            self._save_episode()

                        elif line == 'd':
                            if self.is_recording:
                                self._stop_recording()
                            self._discard_episode()
                            
                        elif line == 'h':
                            self._go_home()

                        elif line == 'n':
                            if self.is_recording:
                                self._save_and_home()
                            else:
                                self._go_home()
                        
                        elif line == 'q':
                            if self.is_recording:
                                self._save_and_home()
                            self.get_logger().info("👋 Quitting...")
                            self.dataset.finalize()
                            rclpy.shutdown()
                            break
                            
                except Exception as e:
                    self.get_logger().debug(f"Keyboard input error: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {str(e)}")
	
    def _mouse_listener(self):
        """键盘监听线程"""
        import sys
        import select
        
        try:
            while rclpy.ok():
                try:
                    for event in self.mouse_keyboard_node.read_loop():
                        if event.type == ecodes.EV_KEY:
                            key_event = categorize(event)
                            if key_event.keystate == key_event.key_down and key_event.scancode==ecodes.KEY_F23:
                                if not self.is_recording:
                                    self._start_recording()
                                else:
                                    self._stop_recording()
                            if key_event.keystate == key_event.key_down and key_event.scancode==ecodes.KEY_F24:
                                if self.is_recording:
                                    self.get_logger().info(
                f"✅ change current phase"
            )
                                    self.current_task_phase+=1
                except Exception as e:
                    self.get_logger().debug(f"Keyboard input error: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {str(e)}")
    def cleanup(self):
        """清理资源"""
        try:
            if self.is_recording:
                self._save_episode()

            # 停止图像写入线程（lerobot 0.3.3 移除了 finalize()）
            self.dataset.stop_image_writer()

            # 用 print 避免在 context 失效后通过 rosout 发布日志触发 C 层报错
            print("✅ Cleanup completed", flush=True)

        except Exception as e:
            print(f"Error during cleanup: {e}", flush=True)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='XArm ROS2 LeRobot Data Collector')
    parser.add_argument('--repo-id', type=str, required=True,
                       help='Dataset repository ID (e.g., myuser/dataset_name)')
    parser.add_argument('--root', type=str, default='~/lerobot_datasets',
                       help='Root directory for datasets')
    parser.add_argument('--fps', type=int, default=30,
                       help='Recording FPS')
    parser.add_argument('--arm-side', type=str, default='right_arm',
                       choices=['right_arm', 'left_arm'],
                       help='Robot arm side: right_arm or left_arm')
    parser.add_argument('--use-wrist-camera', action='store_true', default=False,
                       help='Enable wrist camera for data collection')
    parser.add_argument('--single-task', type=str, default='default_task',
                       help='Task description')
    parser.add_argument('--num-episodes', type=int, default=50,
                       help='Number of episodes to record')
    parser.add_argument('--auto-home-timeout', type=float, default=30.0,
                       help='Timeout for auto home service')
    parser.add_argument('--use-depth-camera', action='store_true', default=False,
                       help='Enable depth camera for data collection')

    args = parser.parse_args()
    # 初始化 ROS2
    rclpy.init()
    
    # 创建采集节点
    collector = XArmROSCollector(
        repo_id=args.repo_id,
        root=args.root,
        fps=args.fps,
        single_task=args.single_task,
        num_episodes=args.num_episodes,
        auto_home_timeout=args.auto_home_timeout,
        arm_side=args.arm_side,
        use_wrist_camera=args.use_wrist_camera,
        use_depth_camera=args.use_depth_camera
    )
    
    try:
        rclpy.spin(collector)
    except Exception as e:
        print(f"setup_fail:{e}")
    finally:
        collector.cleanup()
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

