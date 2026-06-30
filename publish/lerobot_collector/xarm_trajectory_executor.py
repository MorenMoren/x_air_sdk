#!/usr/bin/env python3
"""
XArm Leader 臂控制器 — 通过ROS2话题移动主臂, 从臂自动跟随
===========================================================
遥操作运行中, 发布目标关节位置到此节点, 它只控制Leader臂的CAN总线发送MIT位置命令,
Follower臂通过遥操作控制循环自动跟随, 无需停止遥操作。

原理:
    unilateral_ros2 遥操作运行时:
      - Leader臂: 零力矩/重力补偿模式 (可被拖动, kp=0, kd=0)
      - Follower臂: 位置跟踪模式 (copy Leader位置)
      - sync_references() @ 500Hz: Follower参考 = Leader实际位置 → Follower自动跟随

    本节点:
      1. 只打开 Leader 的 CAN 总线
      2. 接收 /cmd_leader_position 话题 (Float64MultiArray, 7关节值)
      3. 发送 MIT 位置命令 (kp=240, kd=3) 覆盖遥操的零力矩指令
      4. Leader 移动到目标 → 遥操作感知到新位置 → Follower 自动跟随
      5. 轨迹完成后停止发送 → Leader 回到零力矩模式, 停在目标位置

用法:
    # 终端1: 启动遥操作 (正常运行)
    bash start_xarm_teleop.sh unilateral_ros2 right_arm can0 can2

    # 终端2: 启动Leader控制器
    python xarm_leader_controller.py --arm-side right_arm

    # 终端3: 发送目标位置
    ros2 topic pub /cmd_leader_position std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.3, 0.0, 0.8, 0.0, 0.0, 0.9]}"

    # 或使用便捷脚本
    python send_leader_target.py --joints "0.0,0.3,0.0,0.8,0.0,0.0,0.9"

按键:
    '1'/'2'/'3' - 预设测试位置
    'h' - 回到 home 位置
    'q' - 退出
"""

import os
import sys
import time
import signal
import threading
import select
import argparse
import traceback
from pathlib import Path
from typing import Optional, List
import ctypes

import numpy as np
sys.path.append(r"/home/nvidia/x_air_sdk/publish/lerobot_collector/lib")
# import ctypes

# # 1. 加载 .so 库
# # 确保路径正确，如果是当前目录可以写 './libxarm_can_sdk.so'
# ctypes.CDLL('/home/nvidia/x_air_sdk/publish/lerobot_collector/lib/libxarm_can_sdk.so')
import xarm_can as oa

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


# ==============================================================================
# 常量
# ==============================================================================
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

JOINT_LIMITS = {
    'joint1': {'lower': -1.3, 'upper': 3.4},
    'joint2': {'lower': -0.1, 'upper': 1.7},
    'joint3': {'lower': -1.5, 'upper': 1.5},
    'joint4': {'lower':  0.0, 'upper': 2.4},
    'joint5': {'lower': -1.5, 'upper': 1.5},
    'joint6': {'lower': -0.7, 'upper': 0.7},
    'joint7': {'lower': -1.5, 'upper': 1.5},
}

DEFAULT_KP = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0]
DEFAULT_KD = [3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2]

# ★ Leader CAN接口 — 只操作Leader, 不碰Follower
LEADER_CAN_MAP = {
    'right_arm': 'can1',
    'left_arm':  'can0',
}

MOTOR_TYPES = [
    oa.MotorType.DM8009, oa.MotorType.DM8009,
    oa.MotorType.DM4340, oa.MotorType.DM4340,
    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310,
]
SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

DEFAULT_HOME_POSITION = [
    0.09098191559314728, -0.03528648987412453, -0.11234454810619354,
    1.4288166761398315, -0.02079041674733162, 0.024986648932099342,
    0.016594186425209045,
]
DEFAULT_HOME_POSITION_LIST = {'left_arm':[0,0,0,1.6,0,0,0],'right_arm':[0,0,0,1.6,0,0,0]}
#DEFAULT_HOME_POSITION_LIST = {'left_arm':[0,0,-0.1,1.56,-0.23,0,0],'right_arm':[0,0,0,0,0,0,0]}
# ==============================================================================
# Leader CAN 通信层 — 轻量级, 不干扰遥操作
# ==============================================================================
class LeaderCANController:
    """Leader臂CAN通信 — 只做MIT位置发送, 不做电机初始化"""

    def __init__(self, can_if: str, logger):
        self.can_if = can_if
        self.logger = logger
        self.arm: Optional[oa.XArm] = None
        self._connected = False

    def connect(self) -> bool:
        """连接Leader CAN — passive 模式 (SDK-less 原生 socket)

        遥操进程已独占该 CAN 接口的 SDK handle，第二个进程再走 SDK 的
        recv_all 会永久阻塞在初始化握手上。因此这里用 passive 模式：
        读位置靠被动嗅探 STATE 帧，发命令靠直接写原生 CAN 帧，全程不碰
        阻塞的 SDK，可与遥操共存。
        """
        try:
            self.logger.info(f"🔌 连接 Leader CAN: {self.can_if} (passive)")

            # passive=True: 原生 socket, 不创建 SDK handle, 不阻塞
            self.arm = oa.XArm(self.can_if, True, passive=True)

            # 初始化电机对象 (登记 send_id/recv_id, 不发送 CAN 命令)
            self.arm.init_arm_motors(MOTOR_TYPES, SEND_IDS, RECV_IDS)

            # ★ 不调用 enable_all() — 电机已由遥操使能
            # ★ passive 模式下 recv_all 是非阻塞的 socket drain

            # 预热: 嗅探几轮 STATE 帧填充位置缓存
            for _ in range(10):
                self.arm.recv_all()
                time.sleep(0.02)

            motors = self.arm.get_arm().get_motors()
            positions = [m.get_position() for m in motors]
            if any(abs(p) > 1e-6 for p in positions):
                self.logger.info(
                    f"✅ Leader CAN 连接成功 | "
                    f"关节: [{', '.join(f'{p:.3f}' for p in positions[:4])}...]"
                )
                self._connected = True
                return True
            else:
                # 全零: 可能遥操未运行 / 接口无数据。仍标记连接以便发命令，
                # 但提示用户检查。
                self.logger.warning(
                    f"⚠️  Leader CAN {self.can_if} 嗅探到的关节位置全为 0，"
                    "请确认遥操脚本正在运行"
                )
                self._connected = True
                return True

        except Exception as e:
            self.logger.error(f"❌ Leader CAN 连接失败: {e}")
            return False

    def read_positions(self) -> Optional[np.ndarray]:
        """读取Leader当前关节位置"""
        if not self._connected or self.arm is None:
            return None
        try:
            self.arm.refresh_all()
            self.arm.recv_all()
            motors = self.arm.get_arm().get_motors()
            return np.array([m.get_position() for m in motors], dtype=np.float32)
        except Exception:
            return None

    def send_mit_command(self, positions: np.ndarray):
        """发送单帧MIT位置命令到Leader"""
        if not self._connected or self.arm is None:
            return
        params = []
        for i, pos in enumerate(positions[:7]):
            params.append(oa.MITParam(
                DEFAULT_KP[i], DEFAULT_KD[i],
                float(pos), 0.0, 0.0  # pos, vel=0, torque=0
            ))
        self.arm.get_arm().mit_control_all(params)

    def recv(self):
        """接收CAN反馈 (清空接收缓冲)"""
        if self._connected and self.arm is not None:
            try:
                self.arm.recv_all()
            except Exception:
                pass

    def disconnect(self):
        """关闭CAN连接 — 不调用disable_all (遥操需要电机保持使能)"""
        self.logger.info("🔌 释放 Leader CAN 连接")
        self._connected = False
        self.arm = None  # 让GC处理, 不主动disable电机


# ==============================================================================
# 轨迹规划
# ==============================================================================
def plan_trapezoidal(start: np.ndarray, target: np.ndarray,
                     duration_sec: float = 2.2, frequency: float = 200.0
                     ) -> List[np.ndarray]:
    """梯形速度剖面轨迹 (200Hz高频以减少与遥操的竞争)"""
    num_steps = max(20, int(duration_sec * frequency))
    accel_ratio = 0.2
    accel_steps = max(2, int(num_steps * accel_ratio))
    decel_steps = accel_steps
    const_steps = num_steps - accel_steps - decel_steps
    if const_steps < 0:
        accel_steps = num_steps // 2
        decel_steps = num_steps - accel_steps
        const_steps = 0

    trajectory = []
    delta = target - start
    for step in range(num_steps):
        if step < accel_steps:
            t = (step + 1) / accel_steps
            frac = 0.5 * accel_ratio * t * t
        elif const_steps > 0 and step < accel_steps + const_steps:
            t = (step - accel_steps + 1) / const_steps
            frac_start = accel_ratio * 0.5
            frac_range = 1.0 - accel_ratio
            frac = frac_start + frac_range * t
        else:
            t = (step - accel_steps - const_steps + 1) / decel_steps
            frac = 1.0 - 0.5 * accel_ratio * (1.0 - t) * (1.0 - t)
        trajectory.append(start + delta * min(1.0, max(0.0, frac)))
    return trajectory


def clip_joints(positions: np.ndarray) -> np.ndarray:
    clipped = positions.copy()
    for i, name in enumerate(ARM_JOINTS):
        if i >= len(clipped): break
        limits = JOINT_LIMITS[name]
        clipped[i] = np.clip(clipped[i], limits['lower'], limits['upper'])
    return clipped


# ==============================================================================
# ROS2 节点
# ==============================================================================
class LeaderControllerNode(Node):
    """Leader臂ROS2控制节点 — 接收目标→执行轨迹→从臂自动跟随"""

    def __init__(self, arm_side: str = "right_arm", send_freq: float = 200.0,dual_arm:bool=False):
        super().__init__('xarm_leader_controller')
        self.logger.info("🚀 XArm Leader Controller Start")
        self.arm_side = arm_side
        self.send_frequency = send_freq
        self.can_if = LEADER_CAN_MAP[arm_side]
        self.logger = self.get_logger()
        self.dual_arm = dual_arm

        # 控制状态
        self._lock = threading.Lock()
        self._executing = False
        self._trajectory: List[np.ndarray] = []
        self._traj_step = 0
        self._last_position: Optional[np.ndarray] = None

        # CAN通信
        self.can = LeaderCANController(self.can_if, self.logger)

        # ROS2 接口
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            '/cmd_leader_position',
            self._on_target,
            10,
        )
        self.status_pub = self.create_publisher(String, '/cmd_leader_position/status', 10)

        # 控制定时器 (高频发送)
        self.ctrl_timer = self.create_timer(1.0 / send_freq, self._control_loop)

        self._kb_thread = threading.Thread(target=self._keyboard, daemon=True)
        # 启动
        self.logger.info("=" * 55)
        self.logger.info("🚀 XArm Leader Controller Ready")
        self.logger.info("=" * 55)
        self.logger.info(f"  Leader CAN: {self.can_if}")
        self.logger.info(f"  发送频率: {send_freq} Hz")
        self.logger.info(f"  输入话题: /cmd_leader_position")
        self.logger.info("  原理: 控制Leader→遥操作sync_references→Follower自动跟随")
        self.logger.info("=" * 55)

        if not self.can.connect():
            self.logger.error("❌ CAN连接失败! 请确认遥操作在运行且CAN接口正确")
            self.logger.error("   Leader CAN: " + self.can_if)

        self._kb_thread.start()

    # ------------------------------------------------------------------
    def _on_target(self, msg: Float64MultiArray):
        """接收目标关节位置"""
        if len(msg.data) < 7:
            self.logger.error(f"需要7个关节值, 收到 {len(msg.data)}")
            return

        target = clip_joints(np.array(msg.data[:7], dtype=np.float32))
        duration = float(msg.data[7]) if len(msg.data) > 7 else 2.2

        if self._executing:
            self.logger.warn("⚠️  正在执行中, 忽略新目标")
            return

        self.logger.info(f"🎯 收到目标: {np.array2string(target, precision=3)} | {duration}s")

        # 后台线程执行
        threading.Thread(
            target=self._execute, args=(target, duration), daemon=True
        ).start()

    def _execute(self, target: np.ndarray, duration: float):
        """规划并启动轨迹执行"""
        with self._lock:
            if self._executing:
                return
            self._executing = True

        try:
            # 读取Leader当前位置
            current = self.can.read_positions()
            if current is None:
                self.logger.error("❌ 无法读取Leader当前位置")
                return

            self.logger.info(f"📍 起始: {np.array2string(current, precision=3)}")

            if np.allclose(current, target, atol=0.002):
                self.logger.info("✅ 已在目标位置")
                self._publish_status("completed: already at target")
                return

            # 规划轨迹 (高频200Hz)
            traj = plan_trapezoidal(current, target, duration, self.send_frequency)
            self.logger.info(f"📋 轨迹: {len(traj)}步 @ {self.send_frequency}Hz ≈ {duration}s")

            with self._lock:
                self._trajectory = traj
                self._traj_step = 0

            # 等待轨迹执行完成
            while self._executing and self._traj_step < len(traj):
                time.sleep(0.01)

            if self._traj_step >= len(traj):
                final = self.can.read_positions()
                if final is not None:
                    self.logger.info(f"✅ 完成! 最终: {np.array2string(final, precision=3)}")
                self._publish_status("completed")

        except Exception as e:
            self.logger.error(f"❌ 执行失败: {e}")
            traceback.print_exc()
            self._publish_status(f"error: {e}")
        finally:
            with self._lock:
                self._executing = False
                self._trajectory = []
                self._traj_step = 0

    # ------------------------------------------------------------------
    def _control_loop(self):
        """高频控制循环 — 发送MIT位置命令"""
        with self._lock:
            traj = self._trajectory
            step = self._traj_step

        if traj and step < len(traj):
            point = traj[step]
            self.can.send_mit_command(point)
            self.can.recv()
            self._last_position = point

            with self._lock:
                self._traj_step += 1
                new_step = self._traj_step

            if new_step % 50 == 0 or new_step >= len(traj):
                pct = min(100, new_step * 100 // len(traj))
                self.logger.info(f"📊 {new_step}/{len(traj)} ({pct}%) "
                                 f"J1={point[0]:.3f} J4={point[3]:.3f}")
                self._publish_status(f"executing: {new_step}/{len(traj)}")

    # ------------------------------------------------------------------
    def _publish_status(self, status: str):
        try:
            self.status_pub.publish(String(data=status))
        except Exception:
            pass

    # ------------------------------------------------------------------
    def _keyboard(self):
        try:
            while rclpy.ok():
                try:
                    if self.dual_arm:
                        self.logger.info("⌨️  HOME")
                        target = np.array(DEFAULT_HOME_POSITION_LIST[self.arm_side], dtype=np.float32)
                        threading.Thread(
                                target=self._execute, args=(target, 2.0), daemon=True
                            ).start()
                        break
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        line = sys.stdin.readline().strip().lower()
                        if line == 'h':
                            self.logger.info("⌨️  HOME")
                            target = np.array(DEFAULT_HOME_POSITION, dtype=np.float32)
                            threading.Thread(
                                target=self._execute, args=(target, 2.0), daemon=True
                            ).start()
                        elif line == 'q':
                            self.logger.info("👋 退出...")
                            self.cleanup()
                            rclpy.shutdown()
                            break
                except Exception:
                    pass
        except Exception as e:
            self.logger.error(f"键盘错误: {e}")
    def cleanup(self):
        if self.executing:
            time.sleep(1)
            self.logger.info("✅ still executing")
        self._executing = False
        self.can.disconnect()
        self.logger.info("✅ 已清理")

def dualarm_home():
    """双臂归位 — 直接通过CAN发送MIT命令，不创建ROS2节点

    可被外部ROS2节点安全调用 (不调用 rclpy.init())。
    通过 Leader CAN (can1/can2) 将双臂移动到初始位置，
    利用遥操作的 sync_references 机制让 Follower 自动跟随。
    """
    import logging
    _log = logging.getLogger(__name__)

    # 双臂配置
    arm_configs = {
        'left_arm':  {'can': 'can0', 'home': DEFAULT_HOME_POSITION_LIST['left_arm']},
        'right_arm': {'can': 'can1', 'home': DEFAULT_HOME_POSITION_LIST['right_arm']},
    }

    controllers = {}
    start_positions = {}

    try:
        # 1. 连接CAN并读取双臂当前位置
        for arm_name, cfg in arm_configs.items():
            _log.info(f"🔌 连接 {arm_name} Leader CAN: {cfg['can']}")
            ctrl = LeaderCANController(cfg['can'], _log)
            if not ctrl.connect():
                _log.error(f"❌ 无法连接 {arm_name} CAN: {cfg['can']}")
                return
            controllers[arm_name] = ctrl

            pos = ctrl.read_positions()
            if pos is None:
                _log.error(f"❌ 无法读取 {arm_name} 当前位置")
                return
            start_positions[arm_name] = pos
            _log.info(f"📊 {arm_name} 起始位置: {np.array2string(pos, precision=3)}")

        # 2. 规划轨迹 (使用更平滑的参数: 3s, 100Hz)
        send_freq = 100.0
        duration = 2.2
        trajectories = {}
        for arm_name in arm_configs:
            start = start_positions[arm_name]
            target = np.array(arm_configs[arm_name]['home'], dtype=np.float32)
            target = clip_joints(target)
            _log.info(f"🎯 {arm_name} 目标: {np.array2string(target, precision=3)}")
            trajectories[arm_name] = plan_trapezoidal(start, target, duration, send_freq)
            _log.info(f"📋 {arm_name} 轨迹: {len(trajectories[arm_name])}步 @ {send_freq}Hz")

        # 3. 同步执行双臂轨迹
        total_steps = max(len(t) for t in trajectories.values())
        interval = 1.0 / send_freq

        for step in range(total_steps):
            for arm_name in arm_configs:
                traj = trajectories[arm_name]
                if step < len(traj):
                    controllers[arm_name].send_mit_command(traj[step])
                else:
                    # 该臂已到达目标，保持发送最终位置
                    controllers[arm_name].send_mit_command(
                        np.array(arm_configs[arm_name]['home'], dtype=np.float32)
                    )

            # 接收反馈
            for arm_name in arm_configs:
                controllers[arm_name].recv()

            time.sleep(interval)

            if (step + 1) % 50 == 0:
                pct = (step + 1) * 100 // total_steps
                _log.info(f"🏠 Home progress: {pct}%")

        # 4. 验证最终位置
        time.sleep(0.2)
        for arm_name in arm_configs:
            final = controllers[arm_name].read_positions()
            if final is not None:
                _log.info(f"✅ {arm_name} 最终: {np.array2string(final, precision=3)}")

        _log.info("✅ 双臂归位完成")

    except Exception as e:
        _log.error(f"❌ 归位失败: {e}")
        traceback.print_exc()

    finally:
        # 释放CAN连接 (不disable电机)
        for arm_name, ctrl in controllers.items():
            ctrl.disconnect()

# ==============================================================================
