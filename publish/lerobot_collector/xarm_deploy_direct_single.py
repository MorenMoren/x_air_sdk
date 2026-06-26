#!/usr/bin/env python3
"""
XArm 直接硬件部署脚本
命令: s-开始 | S-从初始位置开始 | p-暂停 | h-回初始位置 | q-退出
"""

import os
import sys
sys.path.append(r"/home/nvidia/x_air_sdk/publish/lerobot_collector/lib")
# 直接使用编译好的 xarm_can C++ 扩展
import xarm_can as oa
import rclpy
from rclpy.node import Node
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

# LeRobot 相机接口 (支持新旧版本路径) - 直接通过SDK读取RealSense相机
try:
    from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
    from lerobot.cameras.configs import ColorMode
    _CAMERA_IMPORT_STYLE = 'new'
except ImportError:
    try:
        from lerobot.common.robot_devices.cameras.intelrealsense import RealSenseCamera
        from lerobot.common.robot_devices.cameras.configs import RealSenseCameraConfig, ColorMode  # type: ignore
        _CAMERA_IMPORT_STYLE = 'old'
    except ImportError:
        raise ImportError(
            "Cannot import lerobot camera modules. "
            "Ensure lerobot is installed with: pip install lerobot"
        )


# XArm 关节限制 (rad) - 从 URDF 获取
JOINT_LIMITS = {
    'joint1': {'lower': -1.39, 'upper': 3.49},
    'joint2': {'lower': -1.7, 'upper': 1.7},
    'joint3': {'lower': -1.57, 'upper': 1.57},
    'joint4': {'lower': 0.0, 'upper': 2.4},
    'joint5': {'lower': -1.57, 'upper': 1.57},
    'joint6': {'lower': -0.78, 'upper': 0.78},
    'joint7': {'lower': -1.57, 'upper': 1.57},
}

# 双臂关节限制
LEFT_ARM_JOINT_LIMITS = JOINT_LIMITS.copy()
RIGHT_ARM_JOINT_LIMITS = JOINT_LIMITS.copy()

# 关节名称映射
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
ARM_NAMES = ['left_arm', 'right_arm']

# 夹爪限制 (弧度 rad)
GRIPPER_LIMITS = {
    'lower': -1.0,  # 完全张开 (电机弧度，负值)
    'upper': 0.0,   # 完全闭合
}

# MIT控制参数 - 从硬件接口复制
# 所有臂使用相同参数
DEFAULT_KP = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0]
DEFAULT_KD = [3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2]
GRIPPER_DEFAULT_KP = 16.0
GRIPPER_DEFAULT_KD = 0.3

# CAN接口配置（左右臂）
CAN_INTERFACES = {'left_arm': 'can2', 'right_arm': 'can3'}


class XArmDirectDeployer(Node):
    """XArm 双臂机器人直接硬件部署节点 - 无ROS2控制器"""
    
    def __init__(
        self,
        policy_path: str,
        fps: int = 30,
        device: str = "cpu",
        use_half_precision: bool = False,
        control_frequency: int = 30,
        smoothing_alpha: float = 0.2,
        # RealSense 相机参数 (参考 xarm_ros2_record.py)
        serial_chest: str = '',
        serial_wrist_left: str = '',
        serial_wrist_right: str = '',
        use_wrist_camera: bool = False,
        use_depth_camera: bool = False,
        chest_resolution: tuple = (640, 480),
        wrist_resolution: tuple = (640, 480),
    ):
        super().__init__('xarm_direct_deployer_dual_arm')
        
        # 基本参数
        self.policy_path = Path(policy_path).expanduser().resolve()
        self.fps = fps  # 策略推理频率
        self.control_frequency = control_frequency  # 硬件控制频率
        self.mit_frequency = 500  # MIT底层通信频率（固定500Hz）
        self.device = device
        self.use_half = use_half_precision

        # 双臂配置
        self.arm_names = ARM_NAMES  # ['left_arm', 'right_arm']
        self.can_interfaces = CAN_INTERFACES  # {'left_arm': 'can3', 'right_arm': 'can4'}
        self.arms = {}  # 存储两条臂的控制对象

        # RealSense 相机配置 (参考 xarm_ros2_record.py)
        self._camera_serial_map = {
            'cam_chest': serial_chest,
            'cam_wrist_left': serial_wrist_left,
            'cam_wrist_right': serial_wrist_right,
        }
        self.use_wrist_camera = use_wrist_camera
        self.use_depth_camera = use_depth_camera
        self._chest_resolution = chest_resolution  # (width, height)
        self._wrist_resolution = wrist_resolution
        self.cameras: Dict[str, RealSenseCamera] = {}
        self.wrist_camera_key_list = ['observation.images.cam_wrist_left', 'observation.images.cam_wrist_right']
        
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
        
        # 双臂初始位置 (16维: 左臂7关节 + 右臂7关节 + 左夹爪 + 右夹爪)
        self.initial_positions = {
            'left_arm': [0,0,0,1.6,0,0,0],
            'right_arm': [0,0,0,1.6,0,0,0],
            'left_gripper': -1.0,   # 闭合
            'right_gripper': -1.0,  # 闭合
        }
        
        # 当前观察缓存 (与 xarm_ros2_record.py 数据采集格式对齐)
        self.current_observation = {
            'observation.images.cam_chest': None,
            'observation.state': None,  # 16维: 左臂7关节 + 右臂7关节 + 左夹爪 + 右夹爪
        }
        # 可选相机键 (根据配置动态添加)
        if self.use_wrist_camera:
            for wrist_key in self.wrist_camera_key_list:
                self.current_observation[wrist_key] = None
        if self.use_depth_camera:
            self.current_observation['observation.images.chest_depth'] = None
        
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
            'left_joint1': [],  # 左臂关节1
            'left_joint2': [],  # 左臂关节2
            'right_joint1': [], # 右臂关节1
            'right_joint2': [], # 右臂关节2
            'left_joint1_policy': [],  # 左臂关节1的策略输出（原始）
            'left_joint2_policy': [],  # 左臂关节2的策略输出（原始）
            'right_joint1_policy': [], # 右臂关节1的策略输出（原始）
            'right_joint2_policy': [], # 右臂关节2的策略输出（原始）
            'control_step': [],  # 控制步数
            'inference_step': [],  # 推理步数
        }
        self.control_step_counter = 0  # 全局控制步数计数器
        
        # 平滑滤波器相关 (指数移动平均 EMA)
        self.smoothing_alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))  # 0.0=完全平滑, 1.0=无平滑
        self.last_action = None  # 上一次的平滑输出
        
        # 初始化 CAN 硬件 (两条臂)
        self.get_logger().info("🔌 Initializing dual-arm CAN hardware...")
        try:
            for arm_name in self.arm_names:
                can_interface = self.can_interfaces[arm_name]
                self.get_logger().info(f"   Initializing {arm_name} on {can_interface}...")
                
                arm = oa.XArm(can_interface, True)  # True = CAN-FD
                
                # 初始化7个臂关节电机 (DM8009 x2, DM4340 x2, DM4310 x3)
                motor_types = [
                    oa.MotorType.DM8009, oa.MotorType.DM8009,  # Joint 1-2
                    oa.MotorType.DM4340, oa.MotorType.DM4340,  # Joint 3-4
                    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310  # Joint 5-7
                ]
                send_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
                recv_ids = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
                arm.init_arm_motors(motor_types, send_ids, recv_ids)
                
                # 初始化夹爪电机
                arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
                
                # 设置回调模式并使能电机
                arm.set_callback_mode_all(oa.CallbackMode.STATE)
                self.get_logger().info(f"   ⚡ Enabling motors for {arm_name}...")
                arm.enable_all()
                time.sleep(0.1)
                arm.recv_all()
                
                # 读取初始状态确认连接
                arm.refresh_all()
                arm.recv_all()
                
                arm_motors = arm.get_arm().get_motors()
                self.get_logger().info(f"   📊 {arm_name}: {len(arm_motors)} arm motors initialized")
                for i, motor in enumerate(arm_motors):
                    self.get_logger().info(
                        f"       Motor {i+1}: pos={motor.get_position():.3f} rad, "
                        f"vel={motor.get_velocity():.3f} rad/s"
                    )
                
                gripper_motors = arm.get_gripper().get_motors()
                if gripper_motors:
                    self.get_logger().info(f"   📊 {arm_name}: {len(gripper_motors)} gripper motors")
                    for i, motor in enumerate(gripper_motors):
                        self.get_logger().info(
                            f"       Gripper {i+1}: pos={motor.get_position():.3f} rad"
                        )
                
                # 保存臂控制对象
                self.arms[arm_name] = arm
            
            self.is_initialized = True
            self.get_logger().info("✅ Dual-arm CAN hardware initialized and verified")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize CAN hardware: {e}")
            raise
        
        # 加载策略
        self.policy, self.preprocessor, self.postprocessor = self._load_policy()

        # 初始化 RealSense 相机 (直接通过SDK读取，不再依赖ROS2图像话题)
        self._init_cameras()

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
        self.get_logger().info("🚀 XArm Dual-Arm Direct Hardware Deployer Ready")
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
        self.get_logger().info(f"⚡ Dual-arm direct hardware control - No ROS2 controller delay!")
        self.get_logger().info(f"   Left Arm: {self.can_interfaces['left_arm']}")
        self.get_logger().info(f"   Right Arm: {self.can_interfaces['right_arm']}")
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
            
            # 🔑 校验策略期望的相机与当前配置是否匹配
            self._validate_camera_config(config)

            self.get_logger().info(f"✅ Policy loaded successfully")
            return policy, preprocessor, postprocessor
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load policy: {str(e)}")
            import traceback
            traceback.print_exc()
            raise

    def _validate_camera_config(self, config):
        """校验策略期望的图像特征与当前相机配置是否一致。

        如果策略训练时使用了某相机但部署时未启用该相机（或反之），
        会在启动阶段给出明确提示，避免跑到推理时才 KeyError。
        """
        # 策略期望的所有图像特征键 (如 observation.images.cam_chest, observation.images.cam_wrist_left 等)
        expected_image_keys = getattr(config, 'image_features', [])
        if not expected_image_keys:
            self.get_logger().warn("⚠️  Policy config has no 'image_features'; skipping camera validation.")
            return

        # 部署时实际会提供的相机键
        provided_keys = set(self.current_observation.keys())

        expected_set = set(expected_image_keys)
        missing = expected_set - provided_keys
        extra = provided_keys - expected_set

        if missing:
            self.get_logger().error("=" * 60)
            self.get_logger().error("❌ CAMERA MISMATCH: Policy expects cameras that are NOT configured!")
            self.get_logger().error(f"   Missing keys: {sorted(missing)}")
            self.get_logger().error("")
            if any('wrist' in k for k in missing):
                self.get_logger().error("   💡 The policy was trained WITH wrist cameras.")
                self.get_logger().error("      Launch with: --use_wrist_camera")
                self.get_logger().error("      And provide: --serial_wrist_left <SERIAL> --serial_wrist_right <SERIAL>")
            if any('depth' in k for k in missing):
                self.get_logger().error("   💡 The policy was trained WITH a depth camera.")
                self.get_logger().error("      Launch with: --use_depth_camera")
            if not any(kw in k for kw in ['wrist', 'depth'] for k in missing):
                self.get_logger().error("   💡 Check your camera serial numbers and --use_* flags.")
            self.get_logger().error("=" * 60)
            raise RuntimeError(
                f"Camera configuration mismatch. "
                f"Policy expects {sorted(missing)} but these cameras are not enabled. "
                f"See log above for fixes."
            )

        if extra:
            # 额外提供的相机通常无害（策略不使用它），但值得提醒
            self.get_logger().warn("=" * 60)
            self.get_logger().warn("⚠️  Extra cameras provided that policy does NOT use:")
            self.get_logger().warn(f"   Extra keys: {sorted(extra)}")
            self.get_logger().warn("   These cameras will be captured but ignored during inference.")
            self.get_logger().warn("=" * 60)

        self.get_logger().info("✅ Camera configuration matches policy expectations.")

    def _normalize_gripper_value(self, gripper_value) -> float:
        """标准化夹爪值为float"""
        if isinstance(gripper_value, np.ndarray):
            return float(gripper_value.flat[0])
        return float(gripper_value)
    
    def _init_cameras(self):
        """初始化 Intel RealSense 相机 (参考 xarm_ros2_record.py 的 _init_cameras)

        通过 SDK 直读，不再依赖 ROS2 图像话题:
        - cam_chest (D435):  RGB + 可选深度对齐
        - cam_wrist_left (D405):  RGB only (if use_wrist_camera)
        - cam_wrist_right (D405): RGB only (if use_wrist_camera)
        """
        chest_serial = self._camera_serial_map.get('cam_chest', '')
        wrist_left_serial = self._camera_serial_map.get('cam_wrist_left', '')
        wrist_right_serial = self._camera_serial_map.get('cam_wrist_right', '')

        if not chest_serial:
            self.get_logger().error(
                "❌ No chest camera serial! Use --serial-chest to specify."
            )
            raise RuntimeError("Chest camera serial number required")

        # --- 胸相机 (D435) ---
        chest_width, chest_height = self._chest_resolution
        chest_config = RealSenseCameraConfig(
            serial_number_or_name=chest_serial,
            fps=30,
            width=chest_width,
            height=chest_height,
            color_mode=ColorMode.RGB,
            use_depth=self.use_depth_camera,
        )
        self.cameras['cam_chest'] = RealSenseCamera(chest_config)
        self.cameras['cam_chest'].connect()
        self.get_logger().info(
            f"✅ Chest camera connected (serial={chest_serial}, "
            f"{chest_width}x{chest_height}@30fps, depth={self.use_depth_camera})"
        )

        # --- 手腕相机 (D405) ---
        if self.use_wrist_camera:
            wrist_width, wrist_height = self._wrist_resolution
            for wrist_name, wrist_serial in [
                ('cam_wrist_left', wrist_left_serial),
                ('cam_wrist_right', wrist_right_serial),
            ]:
                if not wrist_serial:
                    self.get_logger().warn(f"⚠️  No serial for {wrist_name}, skipping")
                    continue
                wrist_config = RealSenseCameraConfig(
                    serial_number_or_name=wrist_serial,
                    fps=30,
                    width=wrist_width,
                    height=wrist_height,
                    color_mode=ColorMode.RGB,
                    use_depth=False,
                )
                self.cameras[wrist_name] = RealSenseCamera(wrist_config)
                self.cameras[wrist_name].connect()
                self.get_logger().info(
                    f"✅ {wrist_name} connected (serial={wrist_serial}, "
                    f"{wrist_width}x{wrist_height}@30fps)"
                )

    def _read_camera_frames(self) -> Dict[str, np.ndarray]:
        """从所有已连接相机同步读取最新帧 (参考 xarm_ros2_record.py 的 _read_camera_frames)

        返回 dict:
            'observation.images.cam_chest': (C,H,W) uint8 RGB
            'observation.images.cam_wrist_left': (C,H,W) uint8 RGB (if wrist enabled)
            'observation.images.cam_wrist_right': (C,H,W) uint8 RGB (if wrist enabled)
            'observation.images.chest_depth': (3,H,W) float32 normalized depth (if depth enabled)
        时间戳存到 self.obs_timestamps
        """
        frames = {}
        now = self.get_clock().now()

        for cam_name, camera in self.cameras.items():
            try:
                # 使用 async_read (非阻塞) 读取彩色图像
                result = camera.async_read()

                if self.use_depth_camera and cam_name == 'cam_chest':
                    # chest 相机返回 color 图像，深度需单独读取
                    color_img = result
                    depth_map = camera.read_depth(timeout_ms=0)

                    # RGB: (H,W,C) → (C,H,W)
                    frames['observation.images.cam_chest'] = np.transpose(color_img, (2, 0, 1))
                    self.obs_timestamps['observation.images.cam_chest'] = now

                    if depth_map is not None:
                        depth_float = depth_map.astype(np.float32) / 65535.0
                        depth_3ch = np.stack([depth_float, depth_float, depth_float], axis=0)
                        frames['observation.images.chest_depth'] = depth_3ch
                        self.obs_timestamps['observation.images.chest_depth'] = now
                else:
                    # 手腕相机只返回 color
                    color_img = result
                    if cam_name == 'cam_wrist_left':
                        key = 'observation.images.cam_wrist_left'
                    elif cam_name == 'cam_wrist_right':
                        key = 'observation.images.cam_wrist_right'
                    else:
                        key = 'observation.images.cam_chest'

                    frames[key] = np.transpose(color_img, (2, 0, 1))
                    self.obs_timestamps[key] = now

            except Exception as e:
                self.get_logger().warn(
                    f"Failed to read {cam_name}: {e}",
                    throttle_duration_sec=2.0
                )

        return frames

    def _read_hardware_state(self):
        """读取硬件状态 (两条臂) - 在每次推理前调用"""
        start_time = time.perf_counter()
        
        all_joint_positions = []  # 将所有关节位置合并到一个列表
        
        # 读取两条臂的关节位置
        for arm_name in self.arm_names:
            arm = self.arms[arm_name]
            
            # 刷新并接收状态
            arm.refresh_all()
            arm.recv_all()
            
            # 读取关节位置
            arm_motors = arm.get_arm().get_motors()
            joint_positions = [motor.get_position() for motor in arm_motors]
            all_joint_positions.extend(joint_positions)
        
            gripper_motors = arm.get_gripper().get_motors()
            if gripper_motors:
                gripper_position = gripper_motors[0].get_position()
            else:
                gripper_position = 0.0
            all_joint_positions.append(gripper_position)
        
        # 合并为16维状态 (14关节 + 2夹爪)
        with self.lock:
            self.current_observation['observation.state'] = np.array(
                all_joint_positions,
                dtype=np.float32
            )
            self.obs_timestamps['observation.state'] = self.get_clock().now()
        
        # 记录读取时间
        read_time = time.perf_counter() - start_time
        self.hardware_times.append(read_time)

    def _inference_callback(self):
        """策略推理回调 - 生成目标动作 (频率由fps参数控制)

        数据流 (参考 xarm_ros2_record.py 的 _collect_frame_callback):
            1. 从 CAN 读取双臂关节 + 夹爪数据
            2. 从 RealSense SDK 直读相机图像
            3. 组装 observation 并执行策略推理
        """
        # 🔑 1. 先读取当前硬件状态 (即使不在运行状态也要读取，保持状态更新)
        try:
            self._read_hardware_state()
        except Exception as e:
            self.get_logger().error(f"Hardware read error: {e}", throttle_duration_sec=2.0)
            return

        # 2. 如果未运行，只读取不执行
        if not self.is_running:
            return

        # 🔑 3. 从 RealSense SDK 直读相机图像 (替代 ROS2 图像话题订阅)
        try:
            camera_frames = self._read_camera_frames()
        except Exception as e:
            self.get_logger().error(f"Camera read error: {e}", throttle_duration_sec=2.0)
            return

        with self.lock:
            # 将相机帧合并到 current_observation
            for key, img_array in camera_frames.items():
                self.current_observation[key] = img_array

            # 检查所有必需观察数据是否就绪
            # 必需: cam_chest + state
            required_keys = ['observation.images.cam_chest', 'observation.state']
            if self.use_wrist_camera:
                required_keys.extend(self.wrist_camera_key_list)
            if self.use_depth_camera:
                required_keys.append('observation.images.chest_depth')

            if any(self.current_observation.get(k) is None for k in required_keys):
                missing = [k for k in required_keys if self.current_observation.get(k) is None]
                self.get_logger().warn(
                    f'Missing observations: {missing}',
                    throttle_duration_sec=2.0
                )
                return

            observation = {k: v.copy() for k, v in self.current_observation.items()
                          if k in required_keys}
        
        try:
            # 3. 推理生成新的目标动作
            start_time = time.perf_counter()
            action = self._run_inference(observation)
            inference_time = time.perf_counter() - start_time
            
            # 4. 更新目标动作 - 平滑过渡
            with self.lock:
                if self.current_action is None:
                    # 🔑 首次推理：从当前硬件实际位置作为插值起点，平滑过渡到策略输出
                    # 避免从策略输出直接起跳导致机械臂抽动
                    state = self.current_observation['observation.state']
                    # state 为 16 维: 左臂7关节 + 右臂7关节 + 左夹爪 + 右夹爪
                    # action['action'] 为 14 维: 左臂7关节 + 右臂7关节 (不含夹爪)
                    self.current_action = {
                        'action': np.concatenate([state[:7], state[7:14]]).astype(np.float32),
                        'action.gripper': np.array([float(state[14]), float(state[15])], dtype=np.float32),
                    }
                    self.target_action = action
                    self.interpolation_counter = 0
                    self.get_logger().info(
                        f"🔄 First inference: smoothing from hardware state to policy output "
                        f"(interpolation_steps={self.interpolation_steps})"
                    )
                else:
                    # 使用上次目标作为新起点，设置新目标
                    self.current_action = self.target_action
                    self.target_action = action
                    self.interpolation_counter = 0
                
                # 🔑 记录策略原始输出（仅在正常控制阶段）
                # action['action'] 14维: 左臂7关节 + 右臂7关节
                if 'action' in action and len(action['action']) >= 14:
                    action_vals = action['action']
                    self.joint_data_log['left_joint1_policy'].append(float(action_vals[0]))
                    self.joint_data_log['left_joint2_policy'].append(float(action_vals[1]))
                    self.joint_data_log['right_joint1_policy'].append(float(action_vals[7]))
                    self.joint_data_log['right_joint2_policy'].append(float(action_vals[8]))
                    self.joint_data_log['inference_step'].append(self.inference_count)
            
            self.inference_count += 1
            self.inference_times.append(inference_time)
            
            #if self.inference_count % 30 == 0:
            if True:
                avg_inference = np.mean(self.inference_times[-30:]) * 1000
                avg_hw_read = np.mean(self.hardware_times[-30:]) * 1000 if len(self.hardware_times) >= 30 else 0
                self.get_logger().info(
                    f"Inference #{self.inference_count}: "
                    f"inference={avg_inference:.2f}ms, hw_read={avg_hw_read:.2f}ms, "
                    f"policy_fps={1000.0/avg_inference:.1f}"
                    f"inference action : {action['action'] if 'action' in action else 'N/A'}"
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
                
                # 🔑 记录关节1和关节2的interpolated_action (左右臂，用于可视化)
                if 'action' in interpolated_action:
                    joint_positions = interpolated_action['action']
                    if len(joint_positions) >= 14:
                        self.joint_data_log['timestamps'].append(time.time())
                        self.joint_data_log['left_joint1'].append(float(joint_positions[0]))
                        self.joint_data_log['left_joint2'].append(float(joint_positions[1]))
                        self.joint_data_log['right_joint1'].append(float(joint_positions[7]))
                        self.joint_data_log['right_joint2'].append(float(joint_positions[8]))
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
        """线性插值两个动作 (支持双臂)
        
        Args:
            current: 当前动作
            target: 目标动作
            alpha: 插值系数 (0.0 = current, 1.0 = target)
        
        Returns:
            插值后的动作 (14关节 + 2夹爪)
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
            self.get_logger().info("🔍 FIRST INFERENCE - OBSERVATION DATA (Dual-Arm):")
            for key, value in observation.items():
                self.get_logger().info(f"  {key}: shape={value.shape}, dtype={value.dtype}")
            self.get_logger().info("=" * 60)
        
        # 转换为 torch tensor
        batch = {}
        for key, value in observation.items():
            # 图像数据是 uint8，必须先转为 float32 再归一化到 [0,1]，
            # 否则 preprocessor 做算术运算时会触发 uint8 上溢/下溢。
            if value.dtype == np.uint8:
                value = value.astype(np.float32) / 255.0
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
                
                if len(action_output) == 16:
                    # 16维: 左臂7关节 + 右臂7关节 + 左夹爪 + 右夹爪
                    action['action'] = np.concatenate([action_output[:7], action_output[8:15]], axis=0)
                    action['action.gripper'] = np.array([action_output[7],action_output[15]])
                elif len(action_output) == 14:
                    # 14维: 左臂7关节 + 右臂7关节
                    action['action'] = action_output
                else:
                    action['action'] = action_output
        if self.inference_count % 30 == 0:
            print(f"Raw policy output (post-processed): {action}")
        # 应用位置限幅
        action = self._apply_position_limits(action)
        
        # 应用平滑滤波
        action = self._apply_smoothing_filter(action)
        
        return action

    def _apply_smoothing_filter(self, action: Dict) -> Dict:
        """应用平滑滤波器 (指数移动平均) 到策略输出 - 支持双臂
        
        EMA公式: smoothed = alpha * current + (1 - alpha) * last
        alpha越大，越接近当前值；alpha越小，越平滑
        """
        if self.last_action is None:
            # 首次推理，直接使用当前输出
            self.last_action = {k: np.array(v) if isinstance(v, (list, np.ndarray)) else v 
                               for k, v in action.items()}
            return action
        
        smoothed_action = {}
        
        # 平滑关节位置（仅关节，夹爪不平滑）
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
        """应用关节和夹爪位置限幅 (双臂)"""
        was_clipped = False
        
        # 限幅关节位置 (16维: 左臂7 + 右臂7 + 左夹爪 + 右夹爪)
        if 'action' in action:
            joint_positions = np.array(action['action'], dtype=np.float32)
            
            # 限幅左臂 (索引 0-6)
            for i in range(7):
                if i >= len(joint_positions):
                    break
                joint_name = ARM_JOINTS[i]
                limits = LEFT_ARM_JOINT_LIMITS[joint_name]
                
                original = joint_positions[i]
                joint_positions[i] = np.clip(original, limits['lower'], limits['upper'])
                
                if joint_positions[i] != original:
                    was_clipped = True
                    self.clipped_joints.add(f'left_{joint_name}')
            
            # 限幅右臂 (索引 7-13)
            for i in range(7):
                idx = 7 + i
                if idx >= len(joint_positions):
                    break
                joint_name = ARM_JOINTS[i]
                limits = RIGHT_ARM_JOINT_LIMITS[joint_name]
                
                original = joint_positions[idx]
                joint_positions[idx] = np.clip(original, limits['lower'], limits['upper'])
                
                if joint_positions[idx] != original:
                    was_clipped = True
                    self.clipped_joints.add(f'right_{joint_name}')
            
            action['action'] = joint_positions
        
        # 限幅夹爪位置 (16维: 最后两个元素是左右夹爪)
        if 'action.gripper' in action:
            gripper_values = action['action.gripper']
            if isinstance(gripper_values, np.ndarray) and len(gripper_values) == 2:
                # 限幅左夹爪
                original_left = gripper_values[0]
                clipped_left = np.clip(original_left, GRIPPER_LIMITS['lower'], GRIPPER_LIMITS['upper'])
                
                # 限幅右夹爪
                original_right = gripper_values[1]
                clipped_right = np.clip(original_right, GRIPPER_LIMITS['lower'], GRIPPER_LIMITS['upper'])
                
                if clipped_left != original_left:
                    was_clipped = True
                    self.clipped_joints.add('left_gripper')
                
                if clipped_right != original_right:
                    was_clipped = True
                    self.clipped_joints.add('right_gripper')
                
                action['action.gripper'] = np.array([clipped_left, clipped_right], dtype=np.float32)
            else:
                # 单个夹爪值的向后兼容性
                original = self._normalize_gripper_value(gripper_values)
                clipped = np.clip(original, GRIPPER_LIMITS['lower'], GRIPPER_LIMITS['upper'])
                if clipped != original:
                    was_clipped = True
                    self.clipped_joints.add('gripper')
                action['action.gripper'] = np.array([clipped], dtype=np.float32)
        
        if was_clipped:
            self.clipping_count += 1
        
        return action

    def _send_to_hardware(self, action: Dict):
        """直接发送动作到硬件 (两条臂) - 绕过ROS2控制器"""
        # 发送关节位置命令 (16维: 左臂7 + 右臂7 + 左夹爪 + 右夹爪)
        if 'action' in action:
            joint_positions = action['action']
            
            # 左臂关节 (索引 0-6)
            if len(joint_positions) >= 7:
                left_arm_params = []
                for i, pos in enumerate(joint_positions[:7]):
                    left_arm_params.append(oa.MITParam(
                        DEFAULT_KP[i],
                        DEFAULT_KD[i],
                        float(pos),
                        0.0,  # velocity
                        0.0   # torque
                    ))
                
                left_arm = self.arms['left_arm']
                left_arm.get_arm().mit_control_all(left_arm_params)
                
                # 首次发送时打印信息
                if self.inference_count == 0:
                    self.get_logger().info(
                        f"🎯 Left Arm CAN Control | "
                        f"Pos: [{', '.join(f'{p:.3f}' for p in joint_positions[:7])}] | "
                        f"KP: {DEFAULT_KP} | KD: {DEFAULT_KD}"
                    )
            
            # 右臂关节 (索引 7-13)
            if len(joint_positions) >= 14:
                right_arm_params = []
                for i, pos in enumerate(joint_positions[7:14]):
                    right_arm_params.append(oa.MITParam(
                        DEFAULT_KP[i],
                        DEFAULT_KD[i],
                        float(pos),
                        0.0,  # velocity
                        0.0   # torque
                    ))
                
                right_arm = self.arms['right_arm']
                right_arm.get_arm().mit_control_all(right_arm_params)
                
                # 首次发送时打印信息
                if self.inference_count == 0:
                    self.get_logger().info(
                        f"🎯 Right Arm CAN Control | "
                        f"Pos: [{', '.join(f'{p:.3f}' for p in joint_positions[7:14])}] | "
                        f"KP: {DEFAULT_KP} | KD: {DEFAULT_KD}"
                    )
        
        # 发送夹爪命令 (16维: 最后两个元素是左右夹爪)
        if 'action.gripper' in action:
            gripper_pos = action['action.gripper']
            
            # 如果是数组且有两个元素，分别发送给左右臂
            if isinstance(gripper_pos, np.ndarray) and len(gripper_pos) == 2:
                # 左夹爪
                left_gripper_pos = float(gripper_pos[0])
                left_arm = self.arms['left_arm']
                left_arm.get_gripper().mit_control_all([
                    oa.MITParam(GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, left_gripper_pos, 0.0, 0.0)
                ])
                
                # 右夹爪
                right_gripper_pos = float(gripper_pos[1])
                right_arm = self.arms['right_arm']
                right_arm.get_gripper().mit_control_all([
                    oa.MITParam(GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, right_gripper_pos, 0.0, 0.0)
                ])
                
                if self.inference_count == 0:
                    self.get_logger().info(
                        f"✅ Gripper commands: left={left_gripper_pos:.4f} rad, right={right_gripper_pos:.4f} rad"
                    )
            else:
                # 单个夹爪值的向后兼容性（发送给两个夹爪）
                gripper_value = self._normalize_gripper_value(gripper_pos)
                for arm_name in self.arm_names:
                    arm = self.arms[arm_name]
                    arm.get_gripper().mit_control_all([
                        oa.MITParam(GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, gripper_value, 0.0, 0.0)
                    ])
        
        # 🔑 关键：发送命令后需要等待并接收反馈 (参考遥操作脚本)
        time.sleep(0.0002)  # 200 microseconds
        
        # 接收所有臂的反馈
        for arm_name in self.arm_names:
            self.arms[arm_name].recv_all()

    def _start_execution(self, go_home_first: bool = False):
        """开始执行策略"""
        if go_home_first:
            self.get_logger().info("🎯 Moving to home position before starting...")
            self._go_to_home_position()
            time.sleep(3.0)

        # 🔑 重置策略模型内部状态（时序缓存/action chunking/hidden state）
        # ACT、Diffusion Policy 等策略会在 select_action 中累积历史信息，
        # 不重置会导致重启推理时模型输出受上次会话影响，产生剧烈跳变
        if hasattr(self.policy, 'reset'):
            try:
                self.policy.reset()
                self.get_logger().info("🔄 Policy internal state reset")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Failed to reset policy state: {e}")

        self.is_running = True
        self.inference_count = 0
        self.inference_times = []
        self.last_action = None  # 重置平滑滤波器
        self.current_action = None  # 重置插值起点，避免使用旧推理的动作值
        self.target_action = None  # 重置插值终点
        self.interpolation_counter = 0  # 重置插值计数器
        self.get_logger().info("🚀 Policy execution STARTED (Dual-arm direct hardware mode)")

    def _pause_execution(self):
        """暂停执行"""
        self.is_running = False
        self.get_logger().info("⏸️  Policy execution PAUSED")

    def _go_to_home_position(self):
        """移动两条机械臂到初始位置 - 使用平滑插值 (参考 control.cpp AdjustPosition)"""
        try:
            self.get_logger().info("🏠 Moving both arms to home position with smooth interpolation...")
            was_running = self.is_running
            if was_running:
                self._pause_execution()
            
            # 读取两条臂的当前位置
            current_positions = {}
            for arm_name in self.arm_names:
                self.get_logger().info(f"({arm_name}) is readed")
                arm = self.arms[arm_name]
                
                arm.refresh_all()
                arm.recv_all()
                
                arm_motors = arm.get_arm().get_motors()
                current_arm_positions = [motor.get_position() for motor in arm_motors]
                
                gripper_motors = arm.get_gripper().get_motors()
                current_gripper_position = gripper_motors[0].get_position() if gripper_motors else 0.0
                
                current_positions[arm_name] = {
                    'arm': current_arm_positions,
                    'gripper': current_gripper_position
                }
            
            # 目标位置
            target_positions = {
                'left_arm': self.initial_positions['left_arm'],
                'right_arm': self.initial_positions['right_arm'],
                'left_arm_gripper': float(self.initial_positions['left_gripper']),
                'right_arm_gripper': float(self.initial_positions['right_gripper']),
            }
            
            # 参考 control.cpp AdjustPosition: 220 步插值
            nstep = 220
            kp_arm_temp = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0]
            kd_arm_temp = [3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2]
            kp_hand_temp = 10.0
            kd_hand_temp = 0.5
            
            self.get_logger().info(f"   Interpolating both arms over {nstep} steps (≈ 2.2 seconds)")
            
            for step in range(nstep):
                # 计算插值系数
                alpha = (step + 1) / nstep
                beta = 1.0 - alpha
                
                # 为每条臂构建插值位置
                for arm_name in self.arm_names:
                    arm = self.arms[arm_name]
                    
                    # 插值关节位置
                    target_arm_pos = target_positions[f'{arm_name}']
                    current_arm_pos = current_positions[arm_name]['arm']
                    interpolated_arm_positions = [
                        target_arm_pos[i] * alpha + current_arm_pos[i] * beta
                        for i in range(len(target_arm_pos))
                    ]
                    
                    # 插值夹爪位置
                    target_gripper_pos = float(target_positions[f'{arm_name}_gripper'])
                    current_gripper_pos = float(current_positions[arm_name]['gripper'])
                    interpolated_gripper_position = (
                        target_gripper_pos * alpha + current_gripper_pos * beta
                    )
                    
                    # 记录关节数据
                    if len(interpolated_arm_positions) >= 2 and step % 5 == 0:
                        self.joint_data_log['timestamps'].append(time.time())
                        if arm_name == 'left_arm':
                            self.joint_data_log['left_joint1'].append(float(interpolated_arm_positions[0]))
                            self.joint_data_log['left_joint2'].append(float(interpolated_arm_positions[1]))
                        else:
                            self.joint_data_log['right_joint1'].append(float(interpolated_arm_positions[0]))
                            self.joint_data_log['right_joint2'].append(float(interpolated_arm_positions[1]))
                        self.joint_data_log['control_step'].append(-1)
                    
                    # 构建 MIT 控制参数
                    arm_params = []
                    for i, pos in enumerate(interpolated_arm_positions):
                        arm_params.append(oa.MITParam(
                            kp_arm_temp[i],
                            kd_arm_temp[i],
                            float(pos),
                            0.0,
                            0.0
                        ))
                    
                    gripper_params = [
                        oa.MITParam(kp_hand_temp, kd_hand_temp, 
                                   float(interpolated_gripper_position), 0.0, 0.0)
                    ]
                    
                    # 发送命令
                    arm.get_arm().mit_control_all(arm_params)
                    arm.get_gripper().mit_control_all(gripper_params)
                
                # 每步等待 10ms
                time.sleep(0.01)
                
                # 接收反馈
                for arm_name in self.arm_names:
                    self.arms[arm_name].recv_all()
                
                # 每 44 步打印进度
                if (step + 1) % 44 == 0:
                    self.get_logger().info(f"   Progress: {(step + 1) * 100 // nstep}%")
            
            self.get_logger().info("✅ Both arms reached home position smoothly")
            
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
        """清理资源 — 电机 + 相机"""
        try:
            self.is_running = False
            if self.is_initialized:
                self.get_logger().info("🔌 Disabling motors...")
                for arm_name in self.arm_names:
                    arm = self.arms[arm_name]
                    arm.disable_all()
                    arm.recv_all()
                    self.get_logger().info(f"   ✅ {arm_name} motors disabled")

            # 断开所有 RealSense 相机
            for cam_name, camera in self.cameras.items():
                try:
                    camera.disconnect()
                    self.get_logger().info(f"📷 Disconnected {cam_name}")
                except Exception as e:
                    self.get_logger().warn(f"Error disconnecting {cam_name}: {e}")
            self.cameras.clear()

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
            if self.joint_data_log['left_joint1']:
                import json
                log_file = os.path.expanduser('~/tmp/joint_control_log.json')
                os.makedirs(os.path.dirname(log_file), exist_ok=True)
                with open(log_file, 'w') as f:
                    json.dump(self.joint_data_log, f, indent=2)
                self.get_logger().info(f"💾 Joint control log saved to: {log_file}")
                self.get_logger().info(
                    f"   Total records: {len(self.joint_data_log['left_joint1'])}"
                )
            
            # 禁用电机
            
            
            self.get_logger().info("✅ Cleanup completed")
            
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='XArm Dual-Arm Direct Hardware Deployer - Direct RealSense SDK + CAN Control'
    )
    parser.add_argument('--policy-path', type=str, required=True,
                       help='Path to trained policy checkpoint')
    parser.add_argument('--fps', type=int, default=20,
                       help='Policy inference FPS (default: 30)')
    parser.add_argument('--control-frequency', type=int, default=50,
                       help='Hardware control frequency in Hz (default: 50)')
    parser.add_argument('--device', type=str, default='cuda',
                       choices=['cuda', 'cpu'],
                       help='Device to run inference on')
    parser.add_argument('--use-half', action='store_true',
                       help='Use FP16 for faster inference (CUDA only)')
    parser.add_argument('--no-visualization', action='store_true',
                       help='Disable joint angle visualization')

    # RealSense 相机参数 (参考 xarm_ros2_record.py)
    parser.add_argument('--serial-chest', type=str, default='314422070707',
                       help='Chest camera (D435) serial number')
    parser.add_argument('--serial-wrist-left', type=str, default='412622270856',
                       help='Left wrist camera (D405) serial number')
    parser.add_argument('--serial-wrist-right', type=str, default='230322273759',
                       help='Right wrist camera (D405) serial number')
    parser.add_argument('--use-wrist-camera', action='store_true', default=True,
                       help='Enable wrist cameras for observation')
    parser.add_argument('--use-depth-camera', action='store_true', default=False,
                       help='Enable depth camera for observation')
    parser.add_argument('--chest-width', type=int, default=640,
                       help='Chest camera RGB width (default: 640)')
    parser.add_argument('--chest-height', type=int, default=480,
                       help='Chest camera RGB height (default: 480)')
    parser.add_argument('--wrist-width', type=int, default=640,
                       help='Wrist camera RGB width (default: 640)')
    parser.add_argument('--wrist-height', type=int, default=480,
                       help='Wrist camera RGB height (default: 480)')

    args = parser.parse_args()

    # 初始化 ROS2 (用于节点定时器和spin)
    rclpy.init()

    # 创建双臂部署节点
    deployer = XArmDirectDeployer(
        policy_path=args.policy_path,
        fps=args.fps,
        device=args.device,
        use_half_precision=args.use_half,
        control_frequency=args.control_frequency,
        # RealSense 相机参数
        serial_chest=args.serial_chest,
        serial_wrist_left=args.serial_wrist_left,
        serial_wrist_right=args.serial_wrist_right,
        use_wrist_camera=args.use_wrist_camera,
        use_depth_camera=args.use_depth_camera,
        chest_resolution=(args.chest_width, args.chest_height),
        wrist_resolution=(args.wrist_width, args.wrist_height),
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
