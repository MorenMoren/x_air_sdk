#!/usr/bin/env python3
"""
XArm CAN 数据采集脚本 - LeRobot v3.0 格式
=========================================
通过 CAN 总线直接读取双臂关节数据和夹爪数据，不再依赖 ROS2 关节话题。
相机数据仍从 RealSense 直读。

使用示例:
    # 双臂数据采集 (默认 CAN 接口)
    python xarm_ros2_record.py \
        --repo-id myuser/xarm_dataset \
        --root ~/lerobot_datasets_folder/stepi \
        --num-episodes 50 \
        --use-wrist-camera
        --use-depth-camera


CAN 接口说明 (默认配置):
    - can1: 左臂 Leader   (action 数据源 - 7关节 + 夹爪)
    - can3: 左臂 Follower (observation.state 数据源 - 7关节 + 夹爪)
    - can2: 右臂 Leader   (action 数据源 - 7关节 + 夹爪)
    - can4: 右臂 Follower (observation.state 数据源 - 7关节 + 夹爪)

数据格式 (16维):
    observation.state = [left_joint_0..6, left_gripper, right_joint_0..6, right_gripper]
    action            = [left_joint_0..6, left_gripper, right_joint_0..6, right_gripper]

相机说明:
    - /cam_chest:        胸相机 (RealSense D435, 直读)
    - /cam_wrist_left:   左腕相机 (RealSense D405, 直读, 可选)
    - /cam_wrist_right:  右腕相机 (RealSense D405, 直读, 可选)
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray
import numpy as np
import threading
import time
import argparse
import subprocess
import re
import queue
from sensor_msgs.msg import Image
from pathlib import Path
from typing import Dict, Optional
import logging
import traceback
from datetime import datetime

# LeRobot imports
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ==============================================================================
# Rerun 可视化常量
# ==============================================================================
OBS_PREFIX = "observation."
OBS_STR = "observation"
ACTION_PREFIX = "action."
ACTION = "action"


def _is_scalar(v) -> bool:
    """检查一个值是否为标量 (int, float, 或 0维数组)."""
    if isinstance(v, (int, float, np.floating, np.integer)):
        return True
    if isinstance(v, np.ndarray) and v.ndim == 0:
        return True
    return False


def _ensure_hwc(arr: np.ndarray) -> np.ndarray:
    """将疑似 CHW 排列的图像数组转成 HWC 排列，供 Rerun 使用。"""
    if arr.ndim == 3 and arr.shape[0] in (1, 3, 4) and arr.shape[-1] not in (1, 3, 4):
        return np.transpose(arr, (1, 2, 0))
    return arr

# LeRobot 相机接口 (支持新旧版本路径)
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

from evdev import InputDevice, categorize, ecodes
from xarm_trajectory_executor import *

# xarm_can — CAN 总线直读机械臂关节和夹爪数据
sys.path.append(r"/home/nvidia/x_air_sdk/publish/lerobot_collector/lib")
import xarm_can as oa


# ==============================================================================
# CAN 配置常量
# ==============================================================================

# 电机类型: DM8009×4 (joint1-4) + DM4310×3 (joint5-7)
MOTOR_TYPES = [
    oa.MotorType.DM8009, oa.MotorType.DM8009,
    oa.MotorType.DM4340, oa.MotorType.DM4340,
    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310,
]
SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

# 夹爪电机
GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18
GRIPPER_DM4340_TYPE = oa.MotorType.DM4340  # 部分夹爪使用 DM4340

# CAN 接口默认映射
#   Leader  CAN → 采集 action (主臂关节+夹爪)
#   Follower CAN → 采集 observation.state (从臂关节+夹爪)
DEFAULT_CAN_MAP = {
    'left_leader':   'can1',
    'left_follower': 'can3',
    'right_leader':  'can2',
    'right_follower':'can4',
}


# ==============================================================================
# CAN 读取器 — 只读模式，不发送命令
# ==============================================================================
class CANArmReader:
    """单个 CAN 接口的只读封装 — 读取 7 个关节 + 1 个夹爪位置

    重要:
        - 不调用 enable_all() — 电机已由遥操作使能
        - 不调用 set_callback_mode_all() — 避免干扰遥操作的回调配置
        - 只做 refresh_all() + recv_all() + 读位置
    """

    def __init__(self, can_if: str, name: str, gripper_motor_type=None):
        self.can_if = can_if
        self.name = name
        self.arm: Optional[oa.XArm] = None
        self._connected = False
        self._has_gripper = True
        self._gripper_motor_type = gripper_motor_type or GRIPPER_MOTOR_TYPE
        self._logger = logging.getLogger(f'CANArmReader.{name}')

    def connect(self) -> bool:
        """打开 CAN 接口并初始化电机对象 (只读, 不使能)"""
        try:
            self._logger.info(f"🔌 连接 {self.name} CAN: {self.can_if}")

            self.arm = oa.XArm(self.can_if, True)  # True = CAN-FD

            # 初始化 7 个关节电机
            self.arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)

            # 初始化夹爪电机
            try:
                self.arm.init_gripper_motor(
                    self._gripper_motor_type, GRIPPER_SEND_ID, GRIPPER_RECV_ID
                )
            except Exception:
                # 尝试 DM4340 类型
                try:
                    self.arm.init_gripper_motor(
                        GRIPPER_DM4340_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID
                    )
                except Exception:
                    self._logger.warning(f"⚠️  {self.name}: 夹爪电机初始化失败，将不读取夹爪")
                    self._has_gripper = False

            # 验证连接: 尝试读取电机状态
            self.arm.recv_all()
            self.arm.refresh_all()
            self.arm.recv_all()

            motors = self.arm.get_arm().get_motors()
            if motors:
                positions = [m.get_position() for m in motors]
                self._logger.info(
                    f"✅ {self.name} CAN 连接成功 | "
                    f"关节: [{', '.join(f'{p:.3f}' for p in positions[:4])}...]"
                    f"{' | 夹爪: ✓' if self._has_gripper else ''}"
                )
                self._connected = True
                return True
            else:
                self._logger.error(f"❌ {self.name}: 无法读取电机状态")
                return False

        except Exception as e:
            self._logger.error(f"❌ {self.name} CAN 连接失败: {e}")
            traceback.print_exc()
            return False

    def read_positions(self) -> Optional[np.ndarray]:
        """读取关节 + 夹爪位置

        Returns:
            np.ndarray: [j1..j7, gripper] 共 8 维, 或 None (读取失败)
        """
        if not self._connected or self.arm is None:
            return None
        try:
            self.arm.refresh_all()
            self.arm.recv_all()

            motors = self.arm.get_arm().get_motors()
            joint_positions = np.array([m.get_position() for m in motors], dtype=np.float32)

            if self._has_gripper:
                try:
                    gripper_motor = self.arm.get_gripper().get_motors()
                    gripper_pos = np.array([gripper_motor[0].get_position()], dtype=np.float32)
                except Exception:
                    try:
                        # 备选: 通过 arm 获取夹爪状态
                        gripper_state = self.arm.get_gripper_state()
                        gripper_pos = np.array([gripper_state], dtype=np.float32)
                    except Exception:
                        gripper_pos = np.array([0.0], dtype=np.float32)

                return np.concatenate([joint_positions, gripper_pos])
            else:
                return np.concatenate([joint_positions, np.array([0.0], dtype=np.float32)])

        except Exception as e:
            self._logger.debug(f"{self.name} 读取失败: {e}")
            return None

    def disconnect(self):
        """关闭 CAN 连接 — 不调用 disable_all (遥操作需要电机保持使能)"""
        self._logger.info(f"🔌 释放 {self.name} CAN ({self.can_if})")
        self._connected = False
        self.arm = None  # 让 GC 处理

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def has_gripper(self) -> bool:
        return self._has_gripper

class XArmROSCollector(Node):
    """XArm 机器人 CAN 数据采集节点 — 通过 CAN 直读关节/夹爪, 相机由 lerobot 直读"""

    def __init__(
        self,
        repo_id: str,
        root: str,
        fps: int = 15,
        single_task: str = "default_task",
        num_episodes: int = 50,
        auto_home_timeout: float = 30.0,
        arm_side: str = "right_arm",
        use_wrist_camera: bool = False,
        use_depth_camera: bool = False,
        serial_chest: str = '',
        serial_wrist_left: str = '',
        serial_wrist_right: str = '',
        auto_detect_serials: bool = True,
        chest_resolution: tuple = (1280, 720),
        wrist_resolution: tuple = (848, 480),
        enable_gui: bool = False,
        enable_rerun: bool = False,
        # CAN 接口配置
        can_left_leader: str = '',
        can_left_follower: str = '',
        can_right_leader: str = '',
        can_right_follower: str = '',
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
        self.enable_rerun = enable_rerun

        # 臂侧参数 - 转换为短名（right_arm -> right）
        self.arm_side = arm_side
        self.arm_side_short = "right" if arm_side == "right_arm" else "left"
        self.wrist_camera_key = f'observation.images.cam_wrist_{self.arm_side_short}'
        self.wrist_camera_key_list = ['observation.images.cam_wrist_left','observation.images.cam_wrist_right']
        self.get_logger().info(f"🎯 Arm side: {self.arm_side} (prefix: {self.arm_side_short})")
        self.get_logger().info(f"📷 Use wrist camera: {self.use_wrist_camera}")
        self.get_logger().info(f"🔍 Use depth camera: {self.use_depth_camera}")

        # 控制变量
        self.is_recording = False
        self.episode_buffer = []
        self.current_episode_idx = 0
        self.lock = threading.Lock()
        self.current_task_phase = 1
        self._discarded_episodes: list = []  # 记录被 discard 的 episode 索引
    
        # --- 初始化文件日志 ---
        self._log_dir = Path("~/lerobot_datasets_folder/logs").expanduser().resolve()
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_file = self._log_dir / f"record_{self._session_timestamp}.log"

        self._file_logger = logging.getLogger(f"xarm_record_{self._session_timestamp}")
        self._file_logger.setLevel(logging.DEBUG)
        self._file_logger.handlers.clear()
        fh = logging.FileHandler(str(self._log_file), encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            "%(asctime)s | %(levelname)-7s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        ))
        self._file_logger.addHandler(fh)

        # 写入 session 头信息
        self._file_logger.info("=" * 60)
        self._file_logger.info(f"SESSION START — {self._session_timestamp}")
        self._file_logger.info(f"  repo_id:      {self.repo_id}")
        self._file_logger.info(f"  root:         {self.root}")
        self._file_logger.info(f"  fps:          {self.fps}")
        self._file_logger.info(f"  task:         {self.task}")
        self._file_logger.info(f"  arm_side:     {self.arm_side}")
        self._file_logger.info(f"  wrist_camera: {self.use_wrist_camera}")
        self._file_logger.info(f"  depth_camera: {self.use_depth_camera}")
        self._file_logger.info("=" * 60)

        # --- 初始化 RealSense 相机 ---
        self.cameras: Dict[str, RealSenseCamera] = {}
        self._camera_serial_map: Dict[str, str] = {}  # name -> serial
        self._chest_resolution = chest_resolution  # (width, height)
        self._wrist_resolution = wrist_resolution

        
        self._camera_serial_map = {
            'cam_chest': serial_chest,
            'cam_wrist_left': serial_wrist_left,
            'cam_wrist_right': serial_wrist_right,
        }

        # 初始化各相机
        self._init_cameras()

        # --- 初始化 Rerun 可视化 ---
        self.rr = None
        if self.enable_rerun:
            self._init_rerun()

        # --- 胸部相机监看: ROS2 话题发布 ---
        self.enable_gui = enable_gui
        if self.enable_gui:
            self.get_logger().info(
                "🖥️  Chest monitor topic: /cam_chest/monitor/image_raw"
            )
            self.get_logger().info(
                "    View on laptop: ros2 run rqt_image_view rqt_image_view "
                "/cam_chest/monitor/image_raw"
            )

        # --- 初始化 CAN 读取器 (替代 ROS2 关节/夹爪话题订阅) ---
        # 允许通过参数覆盖默认 CAN 接口
        self._can_if_map = {
            'left_leader':   can_left_leader   or DEFAULT_CAN_MAP['left_leader'],
            'left_follower': can_left_follower or DEFAULT_CAN_MAP['left_follower'],
            'right_leader':  can_right_leader  or DEFAULT_CAN_MAP['right_leader'],
            'right_follower':can_right_follower or DEFAULT_CAN_MAP['right_follower'],
        }
        self._can_readers: Dict[str, CANArmReader] = {}
        self._init_can_readers()

        # 当前帧数据缓存
        #   observation.state: 16维 (左从臂 7关节+夹爪 + 右从臂 7关节+夹爪)
        #   action:            16维 (左主臂 7关节+夹爪 + 右主臂 7关节+夹爪)
        frame_keys = {
            'observation.state': None,
            'action': None,
            "observation.task_phase": np.array([1], dtype=np.int64),
        }
        self.current_frame = frame_keys
        self.frame_timestamps = {}

        # 初始化数据集
        self.dataset = self._create_lerobot_dataset()

        # 设置 ROS 订阅 (仅保留 Joycon 控制)
        self._setup_ros_subscriptions()

        # 定时器 (相机 + CAN 数据采集)
        self.timer = self.create_timer(1.0 / self.fps, self._collect_frame_callback)

        # 键盘监听
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info(f"📁 Dataset path: {self.dataset.root}")
        self.get_logger().info("⌨️  Controls:")
        self.get_logger().info("   'r' - Start/Stop recording episode")
        self.get_logger().info("   'h' - Go home (return to initial position)")
        self.get_logger().info("   'n' - Save episode and go home")
        self.get_logger().info("   'q' - Quit and save")
        self.get_logger().info(f"🔌 CAN interfaces: {self._can_if_map}")
        self.get_logger().info(f"📝 Log file: {self._log_file}")

        # 补充 CAN 信息到文件日志
        self._file_logger.info(f"  CAN interfaces: {self._can_if_map}")
        self._file_logger.info(f"  Dataset path:   {self.dataset.root}")
        self._file_logger.info("=" * 60)

    def _create_lerobot_dataset(self) -> LeRobotDataset:
        """创建 LeRobot 数据集 (16维: 14关节 + 2夹爪)"""

        # 定义特征
        joint_count = 14
        gripper_dim = 2
        total_dim = joint_count + gripper_dim  # 16维
        
        # 定义数据集特征（LeRobot v3.0 格式）
        # ⭐ 夹爪合并到 state 和 action 的最后一维
        features = {
            # 图像观察 - 胸部相机（必需）
            'observation.images.cam_chest': {
                'dtype': 'video',
                'shape': (3, 480, 640),
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
            features['observation.images.chest_depth'] = {
                'dtype': 'video',
                'shape': (3,480, 640),
                'names': ['channel','height', 'width'],
            }
            features['observation.images.left_depth'] = {
                'dtype': 'video',
                'shape': (3,480, 640),
                'names': ['channel','height', 'width'],
            }
            features['observation.images.right_depth'] = {
                'dtype': 'video',
                'shape': (3,480, 640),
                'names': ['channel','height', 'width'],
            }
        
        # 如果使用手腕相机，添加到特征中
        if self.use_wrist_camera:
            for wrist_key in self.wrist_camera_key_list:
                features[wrist_key] = {
                'dtype': 'video',
                'shape': (3, 480, 640),
                'names': ['channel', 'height', 'width'],
            }
        
        # 添加状态和动作特征
        features.update({
            # 状态观察 - 16维 (左臂7关节+夹爪 + 右臂7关节+夹爪)
            'observation.state': {
                'dtype': 'float32',
                'shape': (total_dim,),
                'names': [f'left_joint_{i}' for i in range(7)] + ['gripper_position_left']+[f'right_joint_{i}' for i in range(7)] + ['gripper_position_right'],
            },
            # 动作 - 16维 (左臂7关节+夹爪 + 右臂7关节+夹爪)
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

    # ========== CAN 初始化 ==========

    def _init_can_readers(self):
        """初始化 4 路 CAN 读取器 (只读模式，不干扰遥操作)

        CAN 映射:
            left_leader   → action 左半
            left_follower → observation.state 左半
            right_leader  → action 右半
            right_follower→ observation.state 右半
        """
        self.get_logger().info("🔌 初始化 CAN 读取器 (只读模式)...")
        self.get_logger().info(f"   CAN 接口映射: {self._can_if_map}")

        failed = []
        for role, can_if in self._can_if_map.items():
            reader = CANArmReader(can_if, role)
            if reader.connect():
                self._can_readers[role] = reader
            else:
                failed.append(role)
                self.get_logger().error(f"❌ {role} ({can_if}) 连接失败!")

        if failed:
            self.get_logger().error(f"⚠️  以下 CAN 接口连接失败: {failed}")
        if not self._can_readers:
            raise RuntimeError("❌ 所有 CAN 接口连接失败，无法采集数据!")

        self.get_logger().info(
            f"✅ CAN 读取器就绪: {list(self._can_readers.keys())} "
            f"({len(self._can_readers)}/{len(self._can_if_map)})"
        )

    def _read_can_data(self) -> Dict[str, Optional[np.ndarray]]:
        """从所有 CAN 接口读取关节+夹爪数据

        Returns:
            dict with keys:
                'left_leader':   np.ndarray (8,) or None  — 左主臂关节+夹爪
                'left_follower': np.ndarray (8,) or None  — 左从臂关节+夹爪
                'right_leader':  np.ndarray (8,) or None  — 右主臂关节+夹爪
                'right_follower':np.ndarray (8,) or None  — 右从臂关节+夹爪
        """
        results = {}
        for role, reader in self._can_readers.items():
            results[role] = reader.read_positions()
        return results

    # ========== ROS 订阅 (仅保留 Joycon 控制) ==========

    def _setup_ros_subscriptions(self):
        """设置 ROS2 订阅 (仅 Joycon 控制，不含关节/夹爪 — 关节/夹爪从 CAN 直读)"""
        self.get_logger().info("📡 Subscribing to:")
        self.get_logger().info(f"   Chest camera:  [Direct Read via lerobot]")
        if self.use_wrist_camera:
            self.get_logger().info(f"   Wrist cameras: [Direct Read via lerobot]")
        else:
            self.get_logger().info(f"   Wrist camera:  [Disabled]")
        if self.use_depth_camera:
            self.get_logger().info(f"   Depth:         [Aligned to chest color via lerobot]")
        self.get_logger().info(f"   Joint/Gripper: [CAN direct read]")
        self.get_logger().info(f"     left_leader:  {self._can_if_map['left_leader']}   → action (左半)")
        self.get_logger().info(f"     left_follower:{self._can_if_map['left_follower']}   → observation.state (左半)")
        self.get_logger().info(f"     right_leader: {self._can_if_map['right_leader']}  → action (右半)")
        self.get_logger().info(f"     right_follower:{self._can_if_map['right_follower']} → observation.state (右半)")

        # Joycon 控制
        self.create_subscription(
            Float64MultiArray,
            '/cmd_ctl_left',
            self._joycon_left_callback,
            10
        )
        self.create_subscription(
            Float64MultiArray,
            '/cmd_ctl_right',
            self._joycon_right_callback,
            10
        )

    def _init_cameras(self):
        """初始化 Intel RealSense 相机

        - cam_chest (D435):  RGB + 深度对齐 (if use_depth_camera)
        - cam_wrist_left (D405):  RGB only
        - cam_wrist_right (D405): RGB only
        """
        chest_serial = self._camera_serial_map.get('cam_chest', '')
        wrist_left_serial = self._camera_serial_map.get('cam_wrist_left', '')
        wrist_right_serial = self._camera_serial_map.get('cam_wrist_right', '')

        if not chest_serial:
            self.get_logger().error(
                "❌ No chest camera serial! Use --serial-chest or --auto-detect-serials"
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
            f"{chest_width}x{chest_height}@{self.fps}fps, depth={self.use_depth_camera})"
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
                    use_depth=True,  # D405 不需要深度
                )
                self.cameras[wrist_name] = RealSenseCamera(wrist_config)
                self.cameras[wrist_name].connect()
                self.get_logger().info(
                    f"✅ {wrist_name} connected (serial={wrist_serial}, "
                    f"{wrist_width}x{wrist_height}@{self.fps}fps)"
                )

    # ========== Rerun 可视化 ==========

    def _init_rerun(self):
        """初始化 Rerun SDK，用于实时可视化相机画面和关节数据。"""
        try:
            import rerun as rr
            rr.init("xarm_lerobot_collector", spawn=True)
            rr.log(
                "description",
                rr.TextDocument(
                    "## XArm LeRobot Collector\n"
                    f"- **Arm side**: {self.arm_side}\n"
                    f"- **FPS**: {self.fps}\n"
                    f"- **Wrist camera**: {self.use_wrist_camera}\n"
                    f"- **Depth camera**: {self.use_depth_camera}\n"
                    f"- **Task**: {self.task}\n",
                    media_type=rr.MediaType.MARKDOWN,
                ),
                static=True,
            )
            self.rr = rr
            self.get_logger().info("🖥️  Rerun visualization initialized")
            self._file_logger.info("Rerun visualization initialized")
        except ImportError:
            self.get_logger().error(
                "❌ rerun-sdk not installed. Install with: pip install rerun-sdk"
            )
            self.enable_rerun = False
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Rerun: {e}")
            self.enable_rerun = False

    def log_rerun_data(
        self,
        observation: Optional[Dict[str, np.ndarray]] = None,
        action: Optional[np.ndarray] = None,
        compress_images: bool = True,
    ) -> None:
        """将 observation 和 action 数据实时推送到 Rerun 可视化。

        处理规则:
            - 标量值 (float, int) → ``rr.Scalars``
            - 3D 图像数组 (C,H,W) → 转置为 (H,W,C) → ``rr.Image`` 或 ``rr.EncodedImage``
            - 1D 数组 → 逐元素标量
            - 其他多维数组 → 展平后逐元素标量

        Args:
            observation: 图像帧字典, key 如 'observation.images.cam_chest'
            action: 动作数组 (16维)
            compress_images: 是否 JPEG 压缩以节省带宽
        """
        if not self.enable_rerun or self.rr is None:
            return

        rr = self.rr

        # --- 记录图像 (observation.images.*) ---
        if observation:
            for key, img in observation.items():
                if img is None:
                    continue
                try:
                    arr = np.asarray(img)
                    # 跳过低维数组 (深度图可能已经被处理为3ch)
                    if arr.ndim < 2:
                        continue

                    # 转换 CHW → HWC
                    arr = _ensure_hwc(arr)

                    if arr.ndim == 2:
                        # 单通道深度图或灰度图
                        rr.log(f"cameras/{key}", rr.Image(arr))
                    elif arr.ndim == 3:
                        if compress_images:
                            rr.log(
                                f"cameras/{key}",
                                rr.Image(arr).compress(jpeg_quality=85),
                            )
                        else:
                            rr.log(f"cameras/{key}", rr.Image(arr), static=True)

                except Exception as e:
                    self.get_logger().debug(
                        f"Rerun log image '{key}' failed: {e}"
                    )

        # --- 记录 action 标量 ---
        if action is not None:
            try:
                arr = np.asarray(action).flatten()
                joint_count = 7
                # 左侧 (leader) action 关节
                for i in range(min(joint_count, len(arr))):
                    rr.log(f"action/left_joint_{i}", rr.Scalars(float(arr[i])))
                # 左侧夹爪
                if len(arr) > joint_count:
                    rr.log("action/left_gripper", rr.Scalars(float(arr[joint_count])))
                # 右侧 (leader) action 关节
                offset = joint_count + 1  # 跳过左臂 8 维
                for i in range(joint_count):
                    idx = offset + i
                    if idx < len(arr):
                        rr.log(f"action/right_joint_{i}", rr.Scalars(float(arr[idx])))
                # 右侧夹爪
                gripper_idx = offset + joint_count
                if gripper_idx < len(arr):
                    rr.log("action/right_gripper", rr.Scalars(float(arr[gripper_idx])))
            except Exception as e:
                self.get_logger().debug(f"Rerun log action failed: {e}")

        # --- 记录 observation.state 标量 ---
        state = self.current_frame.get('observation.state')
        if state is not None:
            try:
                arr = np.asarray(state).flatten()
                joint_count = 7
                # 左侧 (follower) state 关节
                for i in range(min(joint_count, len(arr))):
                    rr.log(f"state/left_joint_{i}", rr.Scalars(float(arr[i])))
                # 左侧夹爪
                if len(arr) > joint_count:
                    rr.log("state/left_gripper", rr.Scalars(float(arr[joint_count])))
                # 右侧 (follower) state 关节
                offset = joint_count + 1
                for i in range(joint_count):
                    idx = offset + i
                    if idx < len(arr):
                        rr.log(f"state/right_joint_{i}", rr.Scalars(float(arr[idx])))
                # 右侧夹爪
                gripper_idx = offset + joint_count
                if gripper_idx < len(arr):
                    rr.log("state/right_gripper", rr.Scalars(float(arr[gripper_idx])))
            except Exception as e:
                self.get_logger().debug(f"Rerun log state failed: {e}")

    def _read_camera_frames(self) -> Dict[str, np.ndarray]:
        """从所有已连接相机同步读取最新帧

        返回 dict:
            'observation.images.cam_chest': (C,H,W) uint8 RGB
            'observation.images.cam_wrist_left': (C,H,W) uint8 RGB (if wrist enabled)
            'observation.images.cam_wrist_right': (C,H,W) uint8 RGB (if wrist enabled)
            'observation.depth': (H,W) uint16 (if depth enabled, aligned to chest color)
        时间戳存到 self.frame_timestamps
        """
        frames = {}
        now = self.get_clock().now()

        for cam_name, camera in self.cameras.items():
            try:
                # 优先使用 async_read (非阻塞), 回退到 read() (阻塞)
                result = camera.async_read()
                if self.use_depth_camera:
                    depth_map = camera.read_depth(timeout_ms=0)

                if self.use_depth_camera and cam_name == 'cam_chest':
                    # chest 相机返回 (color, depth) 元组，深度已对齐到彩色
                    color_img = result
                    #depth_map = camera.read_depth(timeout_ms=0)

                    # RGB: (H,W,C) → (C,H,W)
                    frames['observation.images.cam_chest'] = np.transpose(color_img, (2, 0, 1))
                    self.frame_timestamps['observation.images.cam_chest'] = now

                    # 推入发布队列（非阻塞，独立线程负责序列化和 publish）
                    if self.enable_gui and self._chest_pub is not None:
                        try:
                            self._pub_queue.put_nowait(color_img.copy())
                        except queue.Full:
                            pass  # 发布线程来不及消费，丢旧帧

                    if depth_map is not None:
                        depth_float = depth_map.astype(np.float32) / 65535.0
                        depth_3ch = np.stack([depth_float, depth_float, depth_float], axis=0)
                        frames['observation.images.chest_depth'] = depth_3ch
        
                        self.frame_timestamps['observation.images.chest_depth'] = now
                else:
                    # 手腕相机只返回 color
                    color_img = result
                    if self.enable_gui and self._chest_pub is not None and cam_name == 'cam_chest':
                        try:
                            self._pub_queue.put_nowait(color_img.copy())
                        except queue.Full:
                            pass
                    if cam_name == 'cam_wrist_left':
                        
                        key = 'observation.images.cam_wrist_left'
                        
                        #depth_float = depth_map.astype(np.float32) / 65535.0
                        #depth_3ch = np.stack([depth_float, depth_float, depth_float], axis=0)
                        #frames['observation.images.left_depth'] = depth_3ch
        
                        #self.frame_timestamps['observation.images.left_depth'] = now
                    elif cam_name == 'cam_wrist_right':
                        key = 'observation.images.cam_wrist_right'
                        #depth_float = depth_map.astype(np.float32) / 65535.0
                        #depth_3ch = np.stack([depth_float, depth_float, depth_float], axis=0)
                        #frames['observation.images.right_depth'] = depth_3ch
        
                        #self.frame_timestamps['observation.images.right_depth'] = now
                    else:
                        key = 'observation.images.cam_chest'

                    frames[key] = np.transpose(color_img, (2, 0, 1))
                    self.frame_timestamps[key] = now

            except Exception as e:
                self.get_logger().warn(
                    f"Failed to read {cam_name}: {e}",
                    throttle_duration_sec=2.0
                )

        return frames

    # ========== 监看发布线程 ==========

    def _publish_loop(self):
        """独立线程: 从队列取帧 → 序列化 → 发布，不阻塞录制回调"""
        while rclpy.ok():
            try:
                color_img = self._pub_queue.get(timeout=0.5)
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'cam_chest'
                msg.height = color_img.shape[0]
                msg.width = color_img.shape[1]
                msg.encoding = 'rgb8'
                msg.is_bigendian = False
                msg.step = color_img.shape[1] * 3
                msg.data = color_img.tobytes()
                self._chest_pub.publish(msg)
            except queue.Empty:
                continue
            except Exception:
                pass

    # ========== Joycon 控制回调 (保留) ==========

    def _joycon_left_callback(self, msg: Float64MultiArray):
        """左 Joycon 触发: 开始/停止录制"""
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self.lock:
            if not self.is_recording:
                self.get_logger().info("🎮 joycon_left trigger → START recording")
                self._file_logger.info(
                    f"[JOYCON L] action=start | episode={self.current_episode_idx} | timestamp={ts}"
                )
                self._start_recording()
            else:
                self.get_logger().info("🎮 joycon_left trigger → STOP recording")
                self._file_logger.info(
                    f"[JOYCON L] action=stop | episode={self.current_episode_idx} | timestamp={ts}"
                )
                self._stop_recording()

    def _joycon_right_callback(self, msg: Float64MultiArray):
        """右 Joycon 触发: 保存 episode"""
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self.lock:
            self.get_logger().info("🎮 joycon_right trigger → SAVE episode")
            self._file_logger.info(
                f"[JOYCON R] action=save | is_recording={self.is_recording} | "
                f"episode={self.current_episode_idx} | timestamp={ts}"
            )
            if self.is_recording:
                self._stop_recording()
            self._save_episode()

    # ========== CAN 数据采集 (替代 ROS2 关节/夹爪话题) ==========

    def _collect_can_frame(self) -> bool:
        """从 CAN 总线读取双臂关节+夹爪数据并合并到 current_frame

        数据拼装:
            action = [left_leader_joints(7), left_leader_gripper(1),
                      right_leader_joints(7), right_leader_gripper(1)]  → 16维
            observation.state = [left_follower_joints(7), left_follower_gripper(1),
                                 right_follower_joints(7), right_follower_gripper(1)] → 16维

        Returns:
            True 如果所有4路CAN数据都读取成功
        """
        can_data = self._read_can_data()
        now = self.get_clock().now()

        # 检查四路数据完整性
        missing = [role for role, data in can_data.items() if data is None]
        if missing:
            self.get_logger().warn(
                f'CAN read missing: {missing}',
                throttle_duration_sec=2.0
            )
            return False

        left_leader = can_data['left_leader']
        left_follower = can_data['left_follower']
        right_leader = can_data['right_leader']
        right_follower = can_data['right_follower']
        
        #  组装 observation.state: [左从臂 8维] + [右从臂 8维] = 16维
        self.current_frame['observation.state'] = np.concatenate([
            left_follower.astype(np.float32),
            right_follower.astype(np.float32),
        ])
        self.frame_timestamps['observation.state'] = now


        #  组装 action: [左主臂 8维] + [右主臂 8维] = 16维
        self.current_frame['action'] = np.concatenate([
            left_leader.astype(np.float32),
            right_leader.astype(np.float32),
        ])
        self.frame_timestamps['action'] = now

        
        

        return True

    def _collect_frame_callback(self):
        """定时收集帧数据 — 图像从相机直读，关节/夹爪从 CAN 直读"""
        if not self.is_recording:
            return

        with self.lock:
            # 1️⃣ 从 CAN 读取双臂关节 + 夹爪数据
            pre_time = self.get_clock().now()
            can_ok = self._collect_can_frame()
            if not can_ok:
                self.get_logger().warn(
                    'CAN data incomplete, skipping frame',
                    throttle_duration_sec=2.0
                )
                return
            
            # 验证 CAN 数据完整性
            if self.current_frame['action'] is None or self.current_frame['observation.state'] is None:
                self.get_logger().warn(
                    'CAN data not assembled, skipping frame',
                    throttle_duration_sec=2.0
                )
                return
            
            # 2️⃣ 从相机直读图像帧
            
            
            image_frames = self._read_camera_frames()
            current_time_rgb = self.get_clock().now()
            #self.get_logger().info(
             #    f"RGB time consuming {(current_time_rgb - pre_time).nanoseconds / 1e6:.1f} ms"
             #)
            
            # 检查图像数据完整性
            required_image_keys = ['observation.images.cam_chest']
            if self.use_wrist_camera:
                required_image_keys += self.wrist_camera_key_list
            if self.use_depth_camera:
                required_image_keys.append('observation.images.chest_depth')
                required_image_keys.append('observation.images.left_depth')
                required_image_keys.append('observation.images.right_depth')

            missing_images = [k for k in required_image_keys if k not in image_frames]
            if missing_images:
                self.get_logger().warn(
                    f'Missing image data: {missing_images}',
                    throttle_duration_sec=2.0
                )
                return
            
            #with self.lock:
            if True:
            # 构建帧数据
                frame = {
                'observation.images.cam_chest': image_frames['observation.images.cam_chest'].copy(),
                'observation.state': self.current_frame['observation.state'].copy(),
                'action': self.current_frame['action'].copy(),
                "observation.task_phase": np.array([self.current_task_phase], dtype=np.int64),
            }

            # 手腕相机
                if self.use_wrist_camera:
                    for key in self.wrist_camera_key_list:
                        if key in image_frames:
                            frame[key] = image_frames[key].copy()

                # 深度图
                if self.use_depth_camera:
                    frame['observation.images.chest_depth'] = image_frames['observation.images.chest_depth'].copy()
                    frame['observation.images.left_depth'] = image_frames['observation.images.left_depth'].copy()
                    frame['observation.images.right_depth'] = image_frames['observation.images.right_depth'].copy()

                # 添加帧到数据集 buffer
                try:
                    frame["task"] = self.task
                    self.dataset.add_frame(frame)

                    if "size" in self.dataset.episode_buffer:
                        current_frame_count = self.dataset.episode_buffer["size"]
                        if current_frame_count % 30 == 0:
                            self.get_logger().info(
                                f"Episode {self.current_episode_idx}, Frame {current_frame_count}"
                            )

                    # --- Rerun 实时可视化: 每 3 帧推一次以降低开销 ---
                    if self.enable_rerun:
                        frame_count_for_rerun = self.dataset.episode_buffer.get("size", 0)
                        if frame_count_for_rerun % 3 == 0:
                            self.log_rerun_data(
                                observation={
                                    k: v for k, v in frame.items()
                                    if k.startswith('observation.images')
                                },
                                action=frame.get('action'),
                                compress_images=True,
                            )

                except Exception as e:
                    self.get_logger().error(f"❌ Error adding frame: {str(e)}", throttle_duration_sec=1.0)
                    self.is_recording = False
                

    def _go_home(self) -> bool:
        """调用双臂归位 (通过 CAN 发送 MIT 命令)"""
        try:
            self.get_logger().info("🏠 Calling dualarm_home()...")
            dualarm_home()
            self.get_logger().info("🏠 robot go home success")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Error calling home: {str(e)}")
            return False

    def _start_recording(self):
        """开始录制"""
        self.is_recording = True
        self._recording_start_time = time.time()
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        total_saved = self.current_episode_idx
        line = "=" * 50
        self.get_logger().info(line)
        self.get_logger().info(
            f"📹 [REC START] Episode {self.current_episode_idx} | "
            f"task_phase={self.current_task_phase} | "
            f"total_saved={total_saved}"
        )
        self.get_logger().info(line)
        # 文件日志
        self._file_logger.info(line)
        self._file_logger.info(
            f"[REC START] episode={self.current_episode_idx} | "
            f"task_phase={self.current_task_phase} | "
            f"total_saved={total_saved} | "
            f"timestamp={ts}"
        )
        self._file_logger.info(line)

    def _stop_recording(self):
        """停止录制"""
        self.is_recording = False
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # 获取当前 buffer 中的帧数
        frame_count = 0
        if hasattr(self.dataset, 'episode_buffer') and self.dataset.episode_buffer:
            frame_count = self.dataset.episode_buffer.get("size", 0)
        # 计算录制时长
        duration_s = 0.0
        if hasattr(self, '_recording_start_time'):
            duration_s = time.time() - self._recording_start_time
        line = "-" * 40
        self.get_logger().info(line)
        self.get_logger().info(
            f"⏹️  [REC STOP] Episode {self.current_episode_idx} | "
            f"frames_in_buffer={frame_count} | "
            f"duration={duration_s:.1f}s (~{frame_count / self.fps:.1f}s)"
        )
        self.get_logger().info(line)
        # 文件日志
        self._file_logger.info(line)
        self._file_logger.info(
            f"[REC STOP]  episode={self.current_episode_idx} | "
            f"frames_in_buffer={frame_count} | "
            f"duration_s={duration_s:.1f} | "
            f"timestamp={ts}"
        )
        self._file_logger.info(line)
        self.current_task_phase = 1
    def _save_episode(self) -> bool:
        """保存 episode"""
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        save_start = time.time()
        try:
            time.sleep(1)
            # 检查 episode_buffer 是否存在且有数据
            if not hasattr(self.dataset, 'episode_buffer') or not self.dataset.episode_buffer:
                self.get_logger().warn("No episode buffer to save")
                self._file_logger.warning(f"[SAVE FAIL] episode={self.current_episode_idx} | reason=no_buffer | timestamp={ts}")
                return False
            frame_count = self.dataset.episode_buffer.get("size", 0)

            if frame_count == 0:
                self.get_logger().warn("No frames to save")
                self._file_logger.warning(f"[SAVE FAIL] episode={self.current_episode_idx} | reason=zero_frames | timestamp={ts}")
                return False

            self.get_logger().info(
                f"💾 Saving episode {self.current_episode_idx} "
                f"with {frame_count} frames..."
            )

            # 调用 LeRobot 标准保存方法
            save_api_start = time.time()
            self.dataset.save_episode()
            save_api_elapsed = time.time() - save_api_start
            total_elapsed = time.time() - save_start

            self.get_logger().info(
                f"✅ Episode {self.current_episode_idx} saved successfully "
                f"(api={save_api_elapsed:.2f}s, total={total_elapsed:.2f}s)"
            )

            # 文件日志
            self._file_logger.info(
                f"[SAVE OK]   episode={self.current_episode_idx} | "
                f"frames={frame_count} | "
                f"duration_s={frame_count / self.fps:.1f} | "
                f"save_api_s={save_api_elapsed:.2f} | "
                f"save_total_s={total_elapsed:.2f} | "
                f"timestamp={ts}"
            )

            self.current_episode_idx += 1
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Error saving episode: {str(e)}")
            import traceback
            traceback.print_exc()
            self._file_logger.error(
                f"[SAVE ERR]  episode={self.current_episode_idx} | "
                f"error={str(e)} | "
                f"timestamp={ts}"
            )
            return False
    def _discard_episode(self) -> bool:
        """丢弃当前 episode。

        改为调用 save_episode() 正常保存（而非 clear_episode_buffer），
        确保视频编码器正确关闭 segment，避免后续 episode 的时间戳错位。
        录制结束后用 lerobot-edit-dataset 统一删除标记的 episode。
        """
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            time.sleep(1)
            # 检查 episode_buffer 是否存在且有数据
            if not hasattr(self.dataset, 'episode_buffer') or not self.dataset.episode_buffer:
                self.get_logger().warn("No episode buffer to discard")
                self._file_logger.warning(
                    f"[DISCARD FAIL] episode={self.current_episode_idx} | "
                    f"reason=no_buffer | timestamp={ts}"
                )
                return False

            frame_count = self.dataset.episode_buffer.get("size", 0)
            if frame_count == 0:
                self.get_logger().warn("No frames to discard")
                self._file_logger.warning(
                    f"[DISCARD FAIL] episode={self.current_episode_idx} | "
                    f"reason=zero_frames | timestamp={ts}"
                )
                return False

            self.get_logger().info(
                f"🗑️  Discarding episode {self.current_episode_idx} "
                f"({frame_count} frames) — saving it for later deletion..."
            )

            # 正常保存 episode（确保视频 encoder segment 正确关闭）
            save_start = time.time()
            self.dataset.save_episode()
            save_elapsed = time.time() - save_start

            # 记录待删除
            self._discarded_episodes.append(self.current_episode_idx)

            self.get_logger().info(
                f"✅ Episode {self.current_episode_idx} saved (marked for deletion) "
                f"in {save_elapsed:.2f}s"
            )

            # 文件日志
            self._file_logger.info(
                f"[DISCARD]   episode={self.current_episode_idx} | "
                f"frames={frame_count} | "
                f"action=saved_for_deletion | "
                f"save_s={save_elapsed:.2f} | "
                f"total_marked={len(self._discarded_episodes)} | "
                f"timestamp={ts}"
            )

            self.current_episode_idx += 1

            self.current_frame = {
                'observation.state': None,
                'action': None,
                "observation.task_phase": np.array([1], dtype=np.int64),
            }

            return True

        except Exception as e:
            self.get_logger().error(f"❌ Error discarding episode: {str(e)}")
            import traceback
            traceback.print_exc()
            self._file_logger.error(
                f"[DISCARD ERR] episode={self.current_episode_idx} | "
                f"error={str(e)} | timestamp={ts}"
            )
            return False
            
    def _save_and_home(self) -> bool:
        """保存 episode 并复位"""
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._file_logger.info(
            f"[SAVE+HOME] episode={self.current_episode_idx} | "
            f"action=start | timestamp={ts}"
        )

        # 先停止录制
        self.is_recording = False

        # 保存数据
        if not self._save_episode():
            self._file_logger.warning(
                f"[SAVE+HOME FAIL] episode={self.current_episode_idx} | "
                f"reason=save_failed | timestamp={ts}"
            )
            return False

        # 复位机器人
        if self._go_home():
            self.get_logger().info("🆕 Ready for new episode")
            self._file_logger.info(
                f"[SAVE+HOME OK] episode={self.current_episode_idx - 1} | "
                f"home=success | timestamp={ts}"
            )
            return True
        else:
            self.get_logger().warn("⚠️  Failed to go home, please reset manually")
            self._file_logger.warning(
                f"[SAVE+HOME OK] episode={self.current_episode_idx - 1} | "
                f"home=failed | timestamp={ts}"
            )
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
                        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                        if line == 'r':
                            self._file_logger.info(
                                f"[KEY] key=r | is_recording={self.is_recording} | "
                                f"episode={self.current_episode_idx} | timestamp={ts}"
                            )
                            if not self.is_recording:
                                self._start_recording()
                            else:
                                self._stop_recording()

                        elif line == 's':
                            self._file_logger.info(
                                f"[KEY] key=s | is_recording={self.is_recording} | "
                                f"episode={self.current_episode_idx} | timestamp={ts}"
                            )
                            if self.is_recording:
                                self._stop_recording()
                            self._save_episode()

                        elif line == 'd':
                            self._file_logger.info(
                                f"[KEY] key=d | is_recording={self.is_recording} | "
                                f"episode={self.current_episode_idx} | timestamp={ts}"
                            )
                            if self.is_recording:
                                self._stop_recording()
                            self._discard_episode()

                        elif line == 'h':
                            self._file_logger.info(
                                f"[KEY] key=h | episode={self.current_episode_idx} | timestamp={ts}"
                            )
                            self._go_home()

                        elif line == 'n':
                            self._file_logger.info(
                                f"[KEY] key=n | is_recording={self.is_recording} | "
                                f"episode={self.current_episode_idx} | timestamp={ts}"
                            )
                            if self.is_recording:
                                self._save_and_home()
                            else:
                                self._go_home()

                        elif line == 'q':
                            self._file_logger.info(
                                f"[KEY] key=q | is_recording={self.is_recording} | "
                                f"episode={self.current_episode_idx} | timestamp={ts}"
                            )
                            if self.is_recording:
                                self._save_and_home()
                            self.get_logger().info("👋 Quitting...")
                            self.dataset.finalize()
                            # 写入 session 结束信息
                            self._file_logger.info("=" * 60)
                            total_saved = self.current_episode_idx
                            total_discarded = len(self._discarded_episodes)
                            self._file_logger.info(
                                f"SESSION END — total_episodes_saved={total_saved} | "
                                f"total_discards={total_discarded}"
                            )
                            
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
        """清理资源 — CAN 连接 + 相机 + 数据集"""
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            if self.is_recording:
                self._save_episode()

            # 断开所有 CAN 读取器
            for role, reader in self._can_readers.items():
                try:
                    reader.disconnect()
                    print(f"🔌 Disconnected CAN {role} ({reader.can_if})", flush=True)
                except Exception as e:
                    print(f"Error disconnecting CAN {role}: {e}", flush=True)
            self._can_readers.clear()

            # 断开所有相机
            for cam_name, camera in self.cameras.items():
                try:
                    camera.disconnect()
                    print(f"📷 Disconnected {cam_name}", flush=True)
                except Exception as e:
                    print(f"Error disconnecting {cam_name}: {e}", flush=True)
            self.cameras.clear()

            # 销毁监看发布器（发布线程是 daemon，随进程退出）
            # 断开 Rerun 连接
            if self.enable_rerun and self.rr is not None:
                try:
                    self.rr.disconnect()
                    print("🖥️  Rerun visualization disconnected", flush=True)
                except Exception as e:
                    print(f"Error disconnecting Rerun: {e}", flush=True)

            # 停止图像写入线程
            self.dataset.stop_image_writer()

            # 写入 session 结束日志
            total_saved = self.current_episode_idx
            total_discarded = len(self._discarded_episodes)
            self._file_logger.info("=" * 60)
            self._file_logger.info(
                f"SESSION END — {ts} | "
                f"total_episodes_saved={total_saved} | "
                f"total_discards={total_discarded}"
            )
            if self._discarded_episodes:
                # 生成 lerobot-edit 删除命令
                discard_str = " ".join(str(i) for i in self._discarded_episodes)
                dataset_path = self.dataset.root
                delete_cmd = (
                    f"lerobot-edit-dataset "
                    f"--repo_id {dataset_path} "
                    f"--new_repo_id {dataset_path}_cleaned "
                    f"--operation.type delete_episodes "
                    f"--operation.episode_indices \"[{discard_str}]\""
                )
                self._file_logger.info(
                    f"Marked-for-deletion episodes: {self._discarded_episodes}"
                )
                self._file_logger.info(
                    f"To delete them, run: {delete_cmd}"
                )
                print(f"\n{'─'*60}")
                print(f"🗑️  {total_discarded} episode(s) marked for deletion: "
                      f"{self._discarded_episodes}")
                print(f"    录制结束后运行以下命令删除它们：")
                print(f"    {delete_cmd}")
                print(f"{'─'*60}\n")
            self._file_logger.info("=" * 60)

            print("✅ Cleanup completed", flush=True)

        except Exception as e:
            print(f"Error during cleanup: {e}", flush=True)
            try:
                self._file_logger.error(f"[CLEANUP ERR] error={str(e)} | timestamp={ts}")
            except Exception:
                pass


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='XArm CAN LeRobot Data Collector — 通过 CAN 总线直读关节/夹爪数据',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 双臂数据采集 (默认 CAN 接口: can1/can2/can3/can4)
  python xarm_ros2_record.py --repo-id myuser/xarm_dataset --root ~/lerobot_datasets

  # 指定 CAN 接口
  python xarm_ros2_record.py --repo-id myuser/xarm_dataset \\
      --can-left-leader can1 --can-left-follower can3 \\
      --can-right-leader can2 --can-right-follower can4

  # 带手腕相机和深度
  python xarm_ros2_record.py --repo-id myuser/xarm_dataset \\
      --use-wrist-camera --use-depth-camera --num-episodes 50
        """,
    )
    parser.add_argument('--repo-id', type=str, required=True,
                       help='Dataset repository ID (e.g., myuser/dataset_name)')
    parser.add_argument('--root', type=str, default='~/lerobot_datasets',
                       help='Root directory for datasets')
    parser.add_argument('--fps', type=int, default=30,
                       help='Recording FPS')
    parser.add_argument('--arm-side', type=str, default='right_arm',
                       choices=['right_arm', 'left_arm'],
                       help='Robot arm side: right_arm or left_arm (deprecated, CAN reads both arms)')
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
    parser.add_argument('--serial-chest', type=str, default='314422070707',
                       help='Chest camera (D435) serial number')
    parser.add_argument('--serial-wrist-left', type=str, default='412622270856',
                       help='Left wrist camera (D405) serial number')
    parser.add_argument('--serial-wrist-right', type=str, default='230322273759',
                       help='Right wrist camera (D405) serial number')
    parser.add_argument('--no-auto-detect', action='store_true', default=False,
                       help='Disable automatic camera serial detection')
    parser.add_argument('--chest-width', type=int, default=640,
                       help='Chest camera RGB width (default: 640)')
    parser.add_argument('--chest-height', type=int, default=480,
                       help='Chest camera RGB height (default: 720)')
    parser.add_argument('--wrist-width', type=int, default=640,
                       help='Wrist camera RGB width (default: 848)')
    parser.add_argument('--wrist-height', type=int, default=480,
                       help='Wrist camera RGB height (default: 480)')
    # CAN 接口参数
    parser.add_argument('--can-left-leader', type=str, default='can0',
                       help=f'Left leader CAN interface (default: {DEFAULT_CAN_MAP["left_leader"]})')
    parser.add_argument('--can-left-follower', type=str, default='can2',
                       help=f'Left follower CAN interface (default: {DEFAULT_CAN_MAP["left_follower"]})')
    parser.add_argument('--can-right-leader', type=str, default='can1',
                       help=f'Right leader CAN interface (default: {DEFAULT_CAN_MAP["right_leader"]})')
    parser.add_argument('--can-right-follower', type=str, default='can3',
                       help=f'Right follower CAN interface (default: {DEFAULT_CAN_MAP["right_follower"]})')
    parser.add_argument('--enable-rerun', action='store_true', default=False,
                       help='Enable Rerun real-time visualization of camera views and joint data')

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
        use_depth_camera=args.use_depth_camera,
        serial_chest=args.serial_chest,
        serial_wrist_left=args.serial_wrist_left,
        serial_wrist_right=args.serial_wrist_right,
        auto_detect_serials=not args.no_auto_detect,
        chest_resolution=(args.chest_width, args.chest_height),
        wrist_resolution=(args.wrist_width, args.wrist_height),
        can_left_leader=args.can_left_leader,
        can_left_follower=args.can_left_follower,
        can_right_leader=args.can_right_leader,
        can_right_follower=args.can_right_follower,
        enable_rerun=args.enable_rerun,
    )

    # 使用 MultiThreadedExecutor：图像在相机后台线程中读取，
    # 控制数据回调与定时器采集并行
   
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


