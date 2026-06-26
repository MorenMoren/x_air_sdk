#!/usr/bin/env python3
"""
XArm 直接硬件部署脚本
命令: s-开始 | S-从初始位置开始 | p-暂停 | h-回初始位置 | q-退出
"""

import sys
import os

# 直接使用编译好的 xarm_can C++ 扩展
import xarm_can as oa
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import torch
import threading
import time
import argparse
from pathlib import Path
from typing import Dict, Optional
import logging
import select
import traceback

# LeRobot imports
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.configs.policies import PreTrainedConfig

# XArm 关节限制 (rad) - 从 URDF 获取
JOINT_LIMITS = {
    'joint1': {'lower': -1.3, 'upper': 3.4},
    'joint2': {'lower': -0.1, 'upper': 1.7},
    'joint3': {'lower': -1.5, 'upper': 1.5},
    'joint4': {'lower': 0.0, 'upper': 2.4},
    'joint5': {'lower': -1.5, 'upper': 1.5},
    'joint6': {'lower': -0.7, 'upper': 0.7},
    'joint7': {'lower': -1.5, 'upper': 1.5},
}

# 夹爪限制 (弧度 rad)
GRIPPER_LIMITS = {
    'lower': -1.0,  # 完全张开 (电机弧度，负值)
    'upper': 0.0,   # 完全闭合
}

# MIT控制参数 - 从硬件接口复制
DEFAULT_KP = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0]
DEFAULT_KD = [3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2]
GRIPPER_DEFAULT_KP = 16.0
GRIPPER_DEFAULT_KD = 0.3


class XArmDirectDeployer(Node):
    """XArm 机器人直接硬件部署节点 - 无ROS2控制器"""
    
    def __init__(
        self,
        policy_path: str,
        fps: int = 30,
        device: str = "cpu",
        can_interface: str = "can1",
        use_half_precision: bool = False,
        control_frequency: int = 30,
        smoothing_alpha: float = 0.2,
    ):
        super().__init__('xarm_direct_deployer')
        
        # 基本参数
        self.policy_path = Path(policy_path).expanduser().resolve()
        self.fps = fps  # 策略推理频率
        self.control_frequency = control_frequency  # 硬件控制频率
        self.mit_frequency = 500  # MIT底层通信频率（固定500Hz）
        self.device = device
        self.can_interface = can_interface
        self.use_half = use_half_precision
        
        # 计算每个控制周期内的MIT发送次数
        # 例: 500Hz / 50Hz = 10次发送
        self.mit_sends_per_control_cycle = max(1, self.mit_frequency // control_frequency)
        self.mit_interval = 1.0 / self.mit_frequency  # MIT发送间隔（秒）
        
        # 控制变量
        self.is_running = False
        self.is_initialized = False
        self.lock = threading.Lock()
        
        # 插值相关
        self.current_action = None  # 当前动作（插值起点）
        self.target_action = None   # 目标动作（最新推理结果，插值终点）
        self.interpolation_steps = control_frequency // fps  # 插值步数 = 控制频率/推理频率
        self.interpolation_counter = 0  # 当前插值步数计数器
        
        # 机械臂初始位置
        self.initial_positions = {
            'arm': [
                -0.1413366903181501,
                0.14400701915007197,
                -0.2534905012588702,
                0.8703364614328226,
                0.012397955291065799,
                0.12722209506370596,
                0.9061951628900591
            ],
            'gripper': [0.0]  # 闭合
        }
        
        # 当前观察缓存
        self.current_observation = {
            'observation.images.cam_chest': None,
            'observation.images.cam_wrist_right': None,
            'observation.state': None,  # 8维: 7关节 + 1夹爪
        }
        
        # 时间戳记录
        self.obs_timestamps = {}
        
        # 统计信息
        self.inference_count = 0
        self.inference_times = []
        self.hardware_times = []
        
        # 位置限幅统计
        self.clipping_count = 0
        self.clipped_joints = set()
        
        # 关节数据记录 (用于可视化)
        self.joint_data_log = {
            'timestamps': [],
            'joint1': [],  # 关节1的interpolated_action
            'joint2': [],  # 关节2的interpolated_action
            'joint1_policy': [],  # 关节1的策略输出（原始）
            'joint2_policy': [],  # 关节2的策略输出（原始）
            'control_step': [],  # 控制步数
            'inference_step': [],  # 推理步数
        }
        self.control_step_counter = 0  # 全局控制步数计数器
        
        # 平滑滤波器相关 (指数移动平均 EMA)
        self.smoothing_alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))  # 0.0=完全平滑, 1.0=无平滑
        self.last_action = None  # 上一次的平滑输出
        
        # 初始化 CAN 硬件
        self.get_logger().info(f"🔌 Initializing CAN hardware on {can_interface}...")
        try:
            self.arm = oa.XArm(can_interface, True)  # True = CAN-FD
            
            # 初始化7个臂关节电机 (DM8009 x2, DM4340 x2, DM4310 x3)
            motor_types = [
                oa.MotorType.DM8009, oa.MotorType.DM8009,  # Joint 1-2
                oa.MotorType.DM4340, oa.MotorType.DM4340,  # Joint 3-4
                oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310  # Joint 5-7
            ]
            send_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
            recv_ids = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
            self.arm.init_arm_motors(motor_types, send_ids, recv_ids)
            
            # 初始化夹爪电机
            self.arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
            
            # 设置回调模式并使能电机
            self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
            self.get_logger().info("⚡ Enabling motors...")
            self.arm.enable_all()
            time.sleep(0.1)
            self.arm.recv_all()
            
            # 读取初始状态确认连接
            self.arm.refresh_all()
            self.arm.recv_all()
            
            arm_motors = self.arm.get_arm().get_motors()
            self.get_logger().info(f"📊 Arm motors initialized: {len(arm_motors)} motors")
            for i, motor in enumerate(arm_motors):
                self.get_logger().info(
                    f"   Motor {i+1}: pos={motor.get_position():.3f} rad, "
                    f"vel={motor.get_velocity():.3f} rad/s"
                )
            
            gripper_motors = self.arm.get_gripper().get_motors()
            if gripper_motors:
                self.get_logger().info(f"📊 Gripper motors: {len(gripper_motors)} motors")
                for i, motor in enumerate(gripper_motors):
                    self.get_logger().info(
                        f"   Gripper {i+1}: pos={motor.get_position():.3f} rad"
                    )
            
            self.is_initialized = True
            self.get_logger().info("✅ CAN hardware initialized and verified")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize CAN hardware: {e}")
            raise
        
        # 加载策略
        self.policy, self.preprocessor, self.postprocessor = self._load_policy()
        
        # 设置 ROS2 订阅
        self._setup_ros_subscriptions()
        
        # 推理定时器
        self.inference_timer = self.create_timer(1.0 / self.fps, self._inference_callback)
        self.get_logger().info(f"📊 Inference timer: {self.fps} Hz")
        
        # 硬件控制定时器
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self._control_callback)
        self.get_logger().info(f"📊 Control timer: {self.control_frequency} Hz")
        
        # 键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 XArm Direct Hardware Deployer Ready")
        self.get_logger().info("=" * 60)
        self.get_logger().info("⌨️  Controls:")
        self.get_logger().info("   's' - Start policy execution")
        self.get_logger().info("   'S' - Start from home position (capital S)")
        self.get_logger().info("   'p' - Pause execution")
        self.get_logger().info("   'h' - Go to home position")
        self.get_logger().info("   'q' - Quit (will disable motors)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🔧 Smoothing Filter: alpha={self.smoothing_alpha:.2f} (0.0=smooth, 1.0=raw)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"⚡ Direct hardware control - No ROS2 controller delay!")
        self.get_logger().info("=" * 60)

    def _load_policy(self):
        """加载训练好的 LeRobot 策略"""
        try:
            self.get_logger().info(f"📦 Loading policy from: {self.policy_path}")
            
            # 自动检测可用设备
            if self.device == "cuda" and not torch.cuda.is_available():
                self.get_logger().warn("⚠️  CUDA not available. Falling back to CPU.")
                self.device = "cpu"
                self.use_half = False
            
            # 加载配置
            config = PreTrainedConfig.from_pretrained(str(self.policy_path))
            config.device = self.device
            config.pretrained_path = str(self.policy_path)
            
            # 加载策略
            policy_cls = get_policy_class(config.type)
            policy = policy_cls.from_pretrained(
                pretrained_name_or_path=str(self.policy_path),
                config=config,
            )
            
            policy = policy.to(self.device)
            policy.eval()
            
            if self.use_half and self.device == "cuda":
                policy = policy.half()
            
            # 加载处理器
            preprocessor, postprocessor = make_pre_post_processors(
                policy_cfg=config,
                pretrained_path=str(self.policy_path),
                preprocessor_overrides={"device_processor": {"device": self.device}},
                postprocessor_overrides={"device_processor": {"device": self.device}},
            )
            
            self.get_logger().info(f"✅ Policy loaded successfully")
            return policy, preprocessor, postprocessor
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load policy: {str(e)}")
            import traceback
            traceback.print_exc()
            raise

    def _normalize_gripper_value(self, gripper_value) -> float:
        """标准化夹爪值为float"""
        if isinstance(gripper_value, np.ndarray):
            return float(gripper_value.flat[0])
        return float(gripper_value)
    
    def _setup_ros_subscriptions(self):
        """设置 ROS2 订阅 (仅相机)"""
        self.create_subscription(
            Image,
            '/cam_chest/cam_chest/color/image_raw',
            lambda msg: self._image_callback(msg, 'observation.images.cam_chest'),
            10
        )
        
        self.create_subscription(
            Image,
            '/cam_wrist_right/cam_wrist_right/color/image_rect_raw',
            lambda msg: self._image_callback(msg, 'observation.images.cam_wrist_right'),
            10
        )

    def _ros_image_to_numpy(self, msg: Image) -> np.ndarray:
        """将 ROS Image 消息转换为 numpy 数组 (H, W, C) RGB"""
        height, width = msg.height, msg.width
        encoding = msg.encoding
        
        dtype = np.uint8
        if encoding in ['16UC1', 'mono16']:
            dtype = np.uint16
        elif encoding == '32FC1':
            dtype = np.float32
        
        img_array = np.frombuffer(msg.data, dtype=dtype)
        
        if encoding == 'rgb8':
            img = img_array.reshape((height, width, 3))
        elif encoding == 'bgr8':
            img = img_array.reshape((height, width, 3))[:, :, ::-1]
        elif encoding == 'rgba8':
            img = img_array.reshape((height, width, 4))[:, :, :3]
        elif encoding == 'bgra8':
            img = img_array.reshape((height, width, 4))[:, :, [2, 1, 0]]
        elif encoding == 'mono8':
            img = img_array.reshape((height, width, 1))
            img = np.repeat(img, 3, axis=2)
        else:
            self.get_logger().error(f"Unsupported encoding: {encoding}")
            return np.zeros((height, width, 3), dtype=np.uint8)
        
        return img

    def _image_callback(self, msg: Image, key: str):
        """图像回调"""
        with self.lock:
            img = self._ros_image_to_numpy(msg)
            img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
            
            if img.dtype == np.uint8:
                img = img.astype(np.float32) / 255.0
            
            self.current_observation[key] = img
            self.obs_timestamps[key] = self.get_clock().now()

    def _read_hardware_state(self):
        """读取硬件状态 - 在每次推理前调用"""
        start_time = time.perf_counter()
        
        # 刷新并接收所有电机状态
        self.arm.refresh_all()
        self.arm.recv_all()
        
        # 读取关节位置
        arm_motors = self.arm.get_arm().get_motors()
        joint_positions = [motor.get_position() for motor in arm_motors]
        
        # 读取夹爪位置
        gripper_motors = self.arm.get_gripper().get_motors()
        if gripper_motors:
            gripper_position = gripper_motors[0].get_position()
        else:
            gripper_position = 0.0
        
        # 合并为8维状态
        with self.lock:
            self.current_observation['observation.state'] = np.array(
                joint_positions + [gripper_position],
                dtype=np.float32
            )
            self.obs_timestamps['observation.state'] = self.get_clock().now()
        
        # 记录读取时间
        read_time = time.perf_counter() - start_time
        self.hardware_times.append(read_time)

    def _inference_callback(self):
        """策略推理回调 - 生成目标动作 (频率由fps参数控制)"""
        # 🔑 1. 先读取当前硬件状态 (即使不在运行状态也要读取，保持状态更新)
        try:
            self._read_hardware_state()
        except Exception as e:
            self.get_logger().error(f"Hardware read error: {e}", throttle_duration_sec=2.0)
            return
        
        # 2. 如果未运行，只读取不执行
        if not self.is_running:
            return
        
        with self.lock:
            # 检查所有观察数据是否就绪
            if any(v is None for v in self.current_observation.values()):
                missing = [k for k, v in self.current_observation.items() if v is None]
                self.get_logger().warn(
                    f'Missing observations: {missing}',
                    throttle_duration_sec=2.0
                )
                return
            
            observation = {k: v.copy() for k, v in self.current_observation.items()}
        
        try:
            # 3. 推理生成新的目标动作
            start_time = time.perf_counter()
            action = self._run_inference(observation)
            inference_time = time.perf_counter() - start_time
            
            # 4. 更新目标动作 - 平滑过渡
            with self.lock:
                if self.current_action is None:
                    # 首次推理，直接设置
                    self.current_action = action
                    self.target_action = action
                else:
                    # 使用上次目标作为新起点，设置新目标
                    self.current_action = self.target_action
                    self.target_action = action
                    self.interpolation_counter = 0
                
                # 🔑 记录策略原始输出（仅在正常控制阶段）
                if 'action' in action and len(action['action']) >= 2:
                    self.joint_data_log['joint1_policy'].append(float(action['action'][0]))
                    self.joint_data_log['joint2_policy'].append(float(action['action'][1]))
                    self.joint_data_log['inference_step'].append(self.inference_count)
            
            self.inference_count += 1
            self.inference_times.append(inference_time)
            
            if self.inference_count % 30 == 0:
                avg_inference = np.mean(self.inference_times[-30:]) * 1000
                avg_hw_read = np.mean(self.hardware_times[-30:]) * 1000 if len(self.hardware_times) >= 30 else 0
                self.get_logger().info(
                    f"Inference #{self.inference_count}: "
                    f"inference={avg_inference:.2f}ms, hw_read={avg_hw_read:.2f}ms, "
                    f"policy_fps={1000.0/avg_inference:.1f}"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Inference error: {str(e)}")
            import traceback
            traceback.print_exc()

    def _control_callback(self):
        """硬件控制回调 - 执行多次插值发送 (频率由control_frequency参数控制)
        
        新架构：
            - 控制回调频率：control_frequency (e.g., 50Hz)
            - MIT通信频率：mit_frequency (e.g., 500Hz)
            - 每次控制回调：发送 mit_frequency/control_frequency 次命令
            
        例如 50Hz控制 + 500Hz通信:
            - 每个控制周期 20ms
            - 每个MIT间隔 2ms
            - 每次控制回调发送 10 次命令（间隔2ms）
            - 实现高频硬件控制，低频策略推理
        """
        if not self.is_running:
            return
        
        with self.lock:
            # 检查是否有目标动作
            if self.target_action is None or self.current_action is None:
                return
            
            # 生成 mit_sends_per_control_cycle 个插值点并逐一发送
            for send_idx in range(self.mit_sends_per_control_cycle):
                # 计算该发送的细粒度插值系数
                # 从 0.0 到 1.0 均匀分布在控制周期内
                alpha = min(1.0, (self.interpolation_counter + send_idx / self.mit_sends_per_control_cycle) 
                           / self.interpolation_steps)
                
                # 线性插值生成当前控制命令
                interpolated_action = self._interpolate_action(
                    self.current_action,
                    self.target_action,
                    alpha
                )
                
                # 🔑 记录关节1和关节2的interpolated_action (用于可视化)
                if 'action' in interpolated_action:
                    joint_positions = interpolated_action['action']
                    if len(joint_positions) >= 2:
                        self.joint_data_log['timestamps'].append(time.time())
                        self.joint_data_log['joint1'].append(float(joint_positions[0]))
                        self.joint_data_log['joint2'].append(float(joint_positions[1]))
                        self.joint_data_log['control_step'].append(self.control_step_counter)
                
                try:
                    # 发送插值后的命令到硬件
                    hw_start_time = time.perf_counter()
                    self._send_to_hardware(interpolated_action)
                    hardware_time = time.perf_counter() - hw_start_time
                    self.hardware_times.append(hardware_time)
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Control error (send {send_idx+1}/{self.mit_sends_per_control_cycle}): {str(e)}", throttle_duration_sec=1.0)
                
                # 在发送之间等待MIT间隔（最后一次发送不需要等待）
                if send_idx < self.mit_sends_per_control_cycle - 1:
                    time.sleep(self.mit_interval)
            
            # 增加主插值计数器（移动到下一个控制周期）
            self.interpolation_counter += 1
            self.control_step_counter += 1

    def _interpolate_action(self, current: Dict, target: Dict, alpha: float) -> Dict:
        """线性插值两个动作
        
        Args:
            current: 当前动作
            target: 目标动作
            alpha: 插值系数 (0.0 = current, 1.0 = target)
        
        Returns:
            插值后的动作
        """
        interpolated = {}
        beta = 1.0 - alpha
        
        # 插值关节位置
        if 'action' in current and 'action' in target:
            interpolated['action'] = current['action'] * beta + target['action'] * alpha
        
        # 插值夹爪位置
        if 'action.gripper' in current and 'action.gripper' in target:
            current_gripper = current['action.gripper']
            target_gripper = target['action.gripper']
            interpolated['action.gripper'] = current_gripper * beta + target_gripper * alpha
        
        return interpolated

    def _run_inference(self, observation: Dict) -> Dict:
        """执行策略推理"""
        # 首次推理时打印信息
        if self.inference_count == 0:
            self.get_logger().info("=" * 60)
            self.get_logger().info("🔍 FIRST INFERENCE - OBSERVATION DATA:")
            for key, value in observation.items():
                self.get_logger().info(f"  {key}: shape={value.shape}, dtype={value.dtype}")
            self.get_logger().info("=" * 60)
        
        # 转换为 torch tensor
        batch = {}
        for key, value in observation.items():
            tensor = torch.from_numpy(value).unsqueeze(0)
            if self.device == "cuda":
                tensor = tensor.to(self.device)
            if self.use_half and tensor.dtype == torch.float32:
                tensor = tensor.half()
            batch[key] = tensor
        
        # 预处理
        batch = self.preprocessor(batch)
        
        # 推理
        with torch.no_grad():
            action_output = self.policy.select_action(batch)
        
        # 后处理
        action_output = self.postprocessor(action_output)
        
        # 转换回 numpy
        action = {}
        if isinstance(action_output, dict):
            for key, value in action_output.items():
                if isinstance(value, torch.Tensor):
                    action[key] = value.cpu().numpy()[0]
                else:
                    action[key] = value
        else:
            if isinstance(action_output, torch.Tensor):
                action_output = action_output.cpu().numpy()
                while action_output.ndim > 1 and action_output.shape[0] == 1:
                    action_output = action_output[0]
                
                if len(action_output) == 8:
                    action['action'] = action_output[:7]
                    action['action.gripper'] = action_output[7:]
                elif len(action_output) == 7:
                    action['action'] = action_output
                else:
                    action['action'] = action_output
        
        # 应用位置限幅
        action = self._apply_position_limits(action)
        
        # 应用平滑滤波
        action = self._apply_smoothing_filter(action)
        
        return action

    def _apply_smoothing_filter(self, action: Dict) -> Dict:
        """应用平滑滤波器 (指数移动平均) 到策略输出
        
        EMA公式: smoothed = alpha * current + (1 - alpha) * last
        alpha越大，越接近当前值；alpha越小，越平滑
        """
        if self.last_action is None:
            # 首次推理，直接使用当前输出
            self.last_action = {k: np.array(v) if isinstance(v, (list, np.ndarray)) else v 
                               for k, v in action.items()}
            return action
        
        smoothed_action = {}
        
        # 平滑关节位置
        if 'action' in action and 'action' in self.last_action:
            current_joints = np.array(action['action'], dtype=np.float32)
            last_joints = np.array(self.last_action['action'], dtype=np.float32)
            
            # 逐关节平滑
            smoothed_joints = self.smoothing_alpha * current_joints + (1 - self.smoothing_alpha) * last_joints
            smoothed_action['action'] = smoothed_joints
        else:
            smoothed_action['action'] = action.get('action')
        
        # 夹爪不平滑，直接使用原始值
        smoothed_action['action.gripper'] = action.get('action.gripper')
        
        # 保存平滑后的关节位置和原始夹爪位置用于下一帧
        self.last_action = {k: np.array(v) if isinstance(v, (list, np.ndarray)) else v 
                           for k, v in smoothed_action.items()}
        
        return smoothed_action

    def _apply_position_limits(self, action: Dict) -> Dict:
        """应用关节和夹爪位置限幅"""
        was_clipped = False
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # 限幅关节位置
        if 'action' in action:
            joint_positions = np.array(action['action'], dtype=np.float32)
            
            for i, (name, limits) in enumerate(zip(joint_names, JOINT_LIMITS.values())):
                if i >= len(joint_positions):
                    break
                    
                original = joint_positions[i]
                joint_positions[i] = np.clip(original, limits['lower'], limits['upper'])
                
                if joint_positions[i] != original:
                    was_clipped = True
                    self.clipped_joints.add(name)
            
            action['action'] = joint_positions
        
        # 限幅夹爪位置
        if 'action.gripper' in action:
            original_gripper = self._normalize_gripper_value(action['action.gripper'])
            clipped_gripper = np.clip(
                original_gripper,
                GRIPPER_LIMITS['lower'],
                GRIPPER_LIMITS['upper']
            )
            
            if clipped_gripper != original_gripper:
                was_clipped = True
                self.clipped_joints.add('gripper')
            
            # 保持原始类型
            if isinstance(action['action.gripper'], np.ndarray):
                action['action.gripper'] = np.array([clipped_gripper], dtype=np.float32)
            else:
                action['action.gripper'] = clipped_gripper
        
        if was_clipped:
            self.clipping_count += 1
        
        return action

    def _send_to_hardware(self, action: Dict):
        """直接发送动作到硬件 - 绕过ROS2控制器"""
        # 发送关节位置命令
        if 'action' in action:
            joint_positions = action['action']
            
            # 构建MIT控制参数
            arm_params = []
            for i, pos in enumerate(joint_positions[:7]):
                arm_params.append(oa.MITParam(
                    DEFAULT_KP[i],
                    DEFAULT_KD[i],
                    float(pos),
                    0.0,  # velocity
                    0.0   # torque
                ))
            
            # 直接发送到CAN总线
            self.arm.get_arm().mit_control_all(arm_params)
            
            # 首次发送时打印信息
            if self.inference_count == 0:
                self.get_logger().info(
                    f"🎯 Direct CAN Control | "
                    f"Pos: [{', '.join(f'{p:.3f}' for p in joint_positions)}] | "
                    f"KP: {DEFAULT_KP} | KD: {DEFAULT_KD}"
                )
        
        # 发送夹爪命令
        if 'action.gripper' in action:
            gripper_pos = self._normalize_gripper_value(action['action.gripper'])
            
            self.arm.get_gripper().mit_control_all([
                oa.MITParam(
                    GRIPPER_DEFAULT_KP,
                    GRIPPER_DEFAULT_KD,
                    gripper_pos,
                    0.0,
                    0.0
                )
            ])
            
            if self.inference_count == 0:
                self.get_logger().info(f"✅ Gripper command: {gripper_pos:.4f} rad")
        
        # 🔑 关键：发送命令后需要等待并接收反馈 (参考遥操作脚本)
        # 等待 200 微秒让CAN总线处理命令
        time.sleep(0.0002)  # 200 microseconds
        
        # 接收所有电机的反馈
        self.arm.recv_all()

    def _start_execution(self, go_home_first: bool = False):
        """开始执行策略"""
        if go_home_first:
            self.get_logger().info("🎯 Moving to home position before starting...")
            self._go_to_home_position()
            time.sleep(3.0)
        
        self.is_running = True
        self.inference_count = 0
        self.inference_times = []
        self.last_action = None  # 重置平滑滤波器
        self.get_logger().info("🚀 Policy execution STARTED (Direct hardware mode)")

    def _pause_execution(self):
        """暂停执行"""
        self.is_running = False
        self.get_logger().info("⏸️  Policy execution PAUSED")

    def _go_to_home_position(self):
        """移动机械臂到初始位置 - 使用平滑插值 (参考 control.cpp AdjustPosition)"""
        try:
            self.get_logger().info("🏠 Moving to home position with smooth interpolation...")
            was_running = self.is_running
            if was_running:
                self._pause_execution()
            
            # 读取当前位置
            self.arm.refresh_all()
            self.arm.recv_all()
            
            arm_motors = self.arm.get_arm().get_motors()
            current_arm_positions = [motor.get_position() for motor in arm_motors]
            
            gripper_motors = self.arm.get_gripper().get_motors()
            current_gripper_position = gripper_motors[0].get_position() if gripper_motors else 0.0
            
            # 目标位置
            target_arm_positions = self.initial_positions['arm']
            target_gripper_position = self.initial_positions['gripper'][0]
            
            # 参考 control.cpp AdjustPosition: 220 步插值
            nstep = 220
            
            # 参考 control.cpp 的 KP/KD 参数（用于平滑移动）
            # kp_arm_temp = {240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0, 16.0}
            # kd_arm_temp = {3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2, 0.2}
            kp_arm_temp = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0]
            kd_arm_temp = [3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2]
            
            # kp_hand_temp = {10.0}, kd_hand_temp = {0.5}
            kp_hand_temp = 10.0
            kd_hand_temp = 0.5
            # time.sleep(0.01), nstep = 220, 10 * 220 = 2200 ms ≈ 2.2 seconds, 近似于100hz.
            self.get_logger().info(f"   Interpolating over {nstep} steps (≈ 2.2 seconds)")
            
            for step in range(nstep):
                # 计算插值系数 alpha: 从 0 到 1
                alpha = (step + 1) / nstep
                beta = 1.0 - alpha
                
                # 插值计算当前步的目标位置
                interpolated_arm_positions = [
                    target_arm_positions[i] * alpha + current_arm_positions[i] * beta
                    for i in range(len(target_arm_positions))
                ]
                interpolated_gripper_position = (
                    target_gripper_position * alpha + current_gripper_position * beta
                )
                
                # 🔑 记录关节1和关节2的位置到日志（用于可视化移动过程）
                if len(interpolated_arm_positions) >= 2:
                    self.joint_data_log['timestamps'].append(time.time())
                    self.joint_data_log['joint1'].append(float(interpolated_arm_positions[0]))
                    self.joint_data_log['joint2'].append(float(interpolated_arm_positions[1]))
                    # 使用负数表示这是home移动阶段（不是正常控制）
                    self.joint_data_log['control_step'].append(-1)
                
                # 构建 MIT 控制参数（使用临时的较高增益确保跟踪）
                arm_params = []
                for i, pos in enumerate(interpolated_arm_positions):
                    arm_params.append(oa.MITParam(
                        kp_arm_temp[i],
                        kd_arm_temp[i],
                        float(pos),
                        0.0,  # velocity
                        0.0   # torque
                    ))
                
                gripper_params = [
                    oa.MITParam(kp_hand_temp, kd_hand_temp, 
                               float(interpolated_gripper_position), 0.0, 0.0)
                ]
                
                # 发送命令
                self.arm.get_arm().mit_control_all(arm_params)
                self.arm.get_gripper().mit_control_all(gripper_params)
                
                # 每步等待 10ms (参考 control.cpp)
                time.sleep(0.01)
                self.arm.recv_all()
                
                # 每 44 步打印进度（约5次）
                if (step + 1) % 44 == 0:
                    self.get_logger().info(f"   Progress: {(step + 1) * 100 // nstep}%")
            
            self.get_logger().info("✅ Reached home position smoothly")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error going to home: {e}")
            import traceback
            traceback.print_exc()

    def _keyboard_listener(self):
        """键盘监听线程"""
        commands = {
            's': ('start', lambda: self._start_execution(False) if not self.is_running else None),
            'S': ('start_home', lambda: self._start_execution(True) if not self.is_running else None),
            'p': ('pause', lambda: self._pause_execution() if self.is_running else None),
            'h': ('home', self._go_to_home_position),
            'q': ('quit', lambda: (self.get_logger().info("👋 Quitting..."), rclpy.shutdown())),
        }
        
        try:
            while rclpy.ok():
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        line = sys.stdin.readline().strip()
                        
                        if line in commands:
                            _, action = commands[line]
                            action()
                        elif line.lower() in ['s', 'p', 'h', 'q']:
                            # 处理小写版本
                            _, action = commands[line.lower()]
                            action()
                            
                except Exception as e:
                    self.get_logger().debug(f"Keyboard input error: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {e}")

    
    def cleanup(self):
        """清理资源"""
        try:
            self.is_running = False
            
            # 性能统计
            if self.inference_times:
                avg_time = np.mean(self.inference_times) * 1000
                self.get_logger().info(
                    f"📊 Stats: {self.inference_count} inferences, "
                    f"avg {avg_time:.2f}ms ({1000.0/avg_time:.1f} FPS)"
                )
                
                if self.hardware_times:
                    avg_hw = np.mean(self.hardware_times) * 1000
                    self.get_logger().info(f"   Hardware read avg: {avg_hw:.2f}ms")
            
            # 限幅统计
            if self.clipping_count > 0:
                joints_str = ', '.join(sorted(self.clipped_joints))
                self.get_logger().warn(
                    f"⚠️  Clipped {self.clipping_count} times on: {joints_str}"
                )
            
            # 保存关节数据到JSON文件 (用于可视化)
            if self.joint_data_log['joint1']:
                import json
                log_file = '/tmp/joint_control_log.json'
                with open(log_file, 'w') as f:
                    json.dump(self.joint_data_log, f, indent=2)
                self.get_logger().info(f"💾 Joint control log saved to: {log_file}")
                self.get_logger().info(
                    f"   Total records: {len(self.joint_data_log['joint1'])}"
                )
            
            # 禁用电机
            if self.is_initialized:
                self.get_logger().info("🔌 Disabling motors...")
                self.arm.disable_all()
                self.arm.recv_all()
            
            self.get_logger().info("✅ Cleanup completed")
            
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='XArm Direct Hardware Deployer - No ROS2 Controller'
    )
    parser.add_argument('--policy-path', type=str, required=True,
                       help='Path to trained policy checkpoint')
    parser.add_argument('--fps', type=int, default=30,
                       help='Policy inference FPS (default: 30)')
    parser.add_argument('--control-frequency', type=int, default=50,
                       help='Hardware control frequency in Hz (default: 50)')
    parser.add_argument('--device', type=str, default='cpu',
                       choices=['cuda', 'cpu'],
                       help='Device to run inference on')
    parser.add_argument('--can-interface', type=str, default='can1',
                       help='CAN interface name (default: can1)')
    parser.add_argument('--use-half', action='store_true',
                       help='Use FP16 for faster inference (CUDA only)')
    parser.add_argument('--no-visualization', action='store_true',
                       help='Disable joint angle visualization')
    
    args = parser.parse_args()
    
    # 初始化 ROS2 (仅用于相机订阅)
    rclpy.init()
    
    # 创建部署节点
    deployer = XArmDirectDeployer(
        policy_path=args.policy_path,
        fps=args.fps,
        device=args.device,
        can_interface=args.can_interface,
        use_half_precision=args.use_half,
        control_frequency=args.control_frequency,
    )
    
    try:
        rclpy.spin(deployer)
    except KeyboardInterrupt:
        pass
    finally:
        deployer.cleanup()
        deployer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
