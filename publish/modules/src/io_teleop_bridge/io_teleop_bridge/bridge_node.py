"""
io_teleop_bridge 桥接节点
================================

将 TeleXperience 遥操作话题 (/io_teleop/joint_cmd, /io_teleop/target_gripper_status)
桥接到 x_air_sdk 的 ROS2 控制器。

支持两种模式:
  - forward (默认): 转发到 forward_position_controller (直接位置控制，响应最快)
  - trajectory: 转发到 joint_trajectory_controller (轨迹插值，运动更平滑)

支持单臂和双臂自动检测。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np

# ── 关节顺序定义（与 xarm 控制器配置文件一致）───────────────────────
SINGLE_ARM_JOINTS = [
    "xarm_joint1", "xarm_joint2", "xarm_joint3", "xarm_joint4",
    "xarm_joint5", "xarm_joint6", "xarm_joint7",
]
LEFT_ARM_JOINTS = [
    "xarm_left_joint1", "xarm_left_joint2", "xarm_left_joint3",
    "xarm_left_joint4", "xarm_left_joint5", "xarm_left_joint6",
    "xarm_left_joint7",
]
RIGHT_ARM_JOINTS = [
    "xarm_right_joint1", "xarm_right_joint2", "xarm_right_joint3",
    "xarm_right_joint4", "xarm_right_joint5", "xarm_right_joint6",
    "xarm_right_joint7",
]


class IoTeleopBridge(Node):
    """将 /io_teleop/joint_cmd 桥接到 xArm 控制器的 ROS2 节点。"""

    def __init__(self):
        super().__init__("io_teleop_bridge")

        # ── 参数 ────────────────────────────────────────────────
        self.declare_parameter("mode", "forward")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("trajectory_time", 0.5)

        self._mode = self.get_parameter("mode").value
        rate = self.get_parameter("rate").value
        traj_time = self.get_parameter("trajectory_time").value

        self.get_logger().info(f"桥接模式: {self._mode}")

        # ── 状态缓存 ─────────────────────────────────────────────
        self._joint_positions = {}          # name → position 映射
        self._gripper_left = None
        self._gripper_right = None
        self._mode_detected = None          # "single" | "dual" | None
        self._last_traj_goal = None         # 避免重复发送相同 trajectory goal

        # ── 关节顺序索引 ─────────────────────────────────────────
        self._controller_joint_order = []   # 当前控制器的关节顺序

        # ── 发布器 / 客户端 ─────────────────────────────────────
        if self._mode == "forward":
            # forward_position_controller 话题
            self._pub_left = self.create_publisher(
                Float64MultiArray,
                "/left_forward_position_controller/commands", 10,
            )
            self._pub_right = self.create_publisher(
                Float64MultiArray,
                "/right_forward_position_controller/commands", 10,
            )
            self._pub_single = self.create_publisher(
                Float64MultiArray,
                "/forward_position_controller/commands", 10,
            )
        elif self._mode == "trajectory":
            # joint_trajectory_controller Action 客户端
            self._cli_left = ActionClient(
                self, FollowJointTrajectory,
                "/left_joint_trajectory_controller/follow_joint_trajectory",
            )
            self._cli_right = ActionClient(
                self, FollowJointTrajectory,
                "/right_joint_trajectory_controller/follow_joint_trajectory",
            )
            self._cli_single = ActionClient(
                self, FollowJointTrajectory,
                "/joint_trajectory_controller/follow_joint_trajectory",
            )
            # 等待 action server
            for name, cli in [("single", self._cli_single),
                              ("left", self._cli_left),
                              ("right", self._cli_right)]:
                if cli.wait_for_server(timeout_sec=1.0):
                    self.get_logger().info(f"✓ {name} trajectory action server 就绪")
                else:
                    self.get_logger().warn(f"✗ {name} trajectory action server 未响应")

        # ── Gripper Action 客户端 ───────────────────────────────
        self._gripper_left_cli = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd",
        )
        self._gripper_right_cli = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd",
        )
        self._gripper_single_cli = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd",
        )
        for name, cli in [("single", self._gripper_single_cli),
                          ("left", self._gripper_left_cli),
                          ("right", self._gripper_right_cli)]:
            if cli.wait_for_server(timeout_sec=0.5):
                self.get_logger().info(f"✓ {name} gripper action server 就绪")

        # ── 订阅遥操作话题 ───────────────────────────────────────
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )

        self._sub_joint = self.create_subscription(
            JointState, "/io_teleop/joint_cmd", self._on_joint_cmd, qos,
        )
        self._sub_joint_vr = self.create_subscription(
            JointState, "/io_teleop/target_joint_from_vr", self._on_joint_cmd, qos,
        )
        self._sub_gripper = self.create_subscription(
            JointState, "/io_teleop/target_gripper_status",
            self._on_gripper_cmd, qos,
        )

        # 定时器：以固定频率刷新 forward 控制，或发送 trajectory goal
        self.create_timer(1.0 / rate, self._flush)

        self.get_logger().info(
            f"io_teleop_bridge 已启动 (mode={self._mode}, rate={rate}Hz)"
        )
        self.get_logger().info("订阅: /io_teleop/joint_cmd, /io_teleop/target_joint_from_vr, /io_teleop/target_gripper_status")

    # ── 话题回调 ─────────────────────────────────────────────────

    def _on_joint_cmd(self, msg: JointState):
        """缓存最新的关节位置指令。"""
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

        # 自动检测模式：看关节名里有没有 left/right
        if self._mode_detected is None:
            has_left = any("left" in n for n in msg.name)
            has_right = any("right" in n for n in msg.name)
            if has_left or has_right:
                self._mode_detected = "dual"
                self.get_logger().info("自动检测: 双臂模式 (dual)")
            else:
                self._mode_detected = "single"
                self.get_logger().info("自动检测: 单臂模式 (single)")

    def _on_gripper_cmd(self, msg: JointState):
        """缓存夹爪状态指令。"""
        if len(msg.position) > 0:
            self._gripper_left = float(msg.position[0])
        if len(msg.position) > 1:
            self._gripper_right = float(msg.position[1])

    # ── 定时刷新 ─────────────────────────────────────────────────

    def _flush(self):
        """定时器回调：将缓存的位置刷到控制器。"""
        if self._mode_detected is None:
            return  # 还没收到任何关节指令，跳过

        if self._mode == "forward":
            self._flush_forward()
        elif self._mode == "trajectory":
            self._flush_trajectory()

        # 夹爪命令只要有新值就发
        self._flush_gripper()

    # ── Forward 模式 ────────────────────────────────────────────

    def _flush_forward(self):
        """构造关节顺序数组并发布到 forward_position_controller。"""
        if self._mode_detected == "single":
            arr = self._build_float64_array(SINGLE_ARM_JOINTS)
            if arr is not None:
                self._pub_single.publish(arr)
        else:
            larr = self._build_float64_array(LEFT_ARM_JOINTS)
            if larr is not None:
                self._pub_left.publish(larr)
            rarr = self._build_float64_array(RIGHT_ARM_JOINTS)
            if rarr is not None:
                self._pub_right.publish(rarr)

    def _build_float64_array(self, joint_names):
        """按关节顺序从缓存中提取位置，构造 Float64MultiArray。"""
        values = []
        for name in joint_names:
            if name in self._joint_positions:
                values.append(self._joint_positions[name])
            else:
                # 缺关节：用 NaN 会让 controller 报错，跳过本次发送
                return None
        msg = Float64MultiArray()
        msg.data = values
        return msg

    # ── Trajectory 模式 ─────────────────────────────────────────

    def _flush_trajectory(self):
        """构造轨迹目标并发送到 joint_trajectory_controller。"""
        if self._mode_detected == "single":
            self._send_traj_goal(self._cli_single, SINGLE_ARM_JOINTS)
        else:
            self._send_traj_goal(self._cli_left, LEFT_ARM_JOINTS)
            self._send_traj_goal(self._cli_right, RIGHT_ARM_JOINTS)

    def _send_traj_goal(self, client, joint_names):
        """发送 FollowJointTrajectory action goal。"""
        if not client.wait_for_server(timeout_sec=0.0):
            return

        positions = []
        for name in joint_names:
            if name not in self._joint_positions:
                return  # 不完整，跳过
            positions.append(self._joint_positions[name])

        # 跳过重复（和上次完全一样就不发了）
        key = (tuple(joint_names), tuple(round(p, 6) for p in positions))
        if self._last_traj_goal == key:
            return
        self._last_traj_goal = key

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(joint_names)
        goal_msg.trajectory.points.append(
            JointTrajectoryPoint(
                positions=positions,
                time_from_start=rclpy.duration.Duration(
                    seconds=self.get_parameter("trajectory_time").value
                ).to_msg(),
            )
        )

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._traj_goal_response(f, joint_names[0]))

    def _traj_goal_response(self, future, side_prefix):
        """处理 trajectory goal 的响应（仅日志）。"""
        try:
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warn(f"{side_prefix} trajectory goal 被拒绝")
        except Exception as e:
            self.get_logger().error(f"{side_prefix} trajectory goal 失败: {e}")

    # ── Gripper 桥接 ────────────────────────────────────────────

    def _flush_gripper(self):
        """将夹爪状态发到 gripper action controller。"""
        if self._mode_detected == "single":
            if self._gripper_left is not None:
                self._send_gripper_goal(
                    self._gripper_single_cli, self._gripper_left,
                )
                self._gripper_left = None  # 清掉，避免重复发送
        else:
            if self._gripper_left is not None:
                self._send_gripper_goal(
                    self._gripper_left_cli, self._gripper_left,
                )
                self._gripper_left = None
            if self._gripper_right is not None:
                self._send_gripper_goal(
                    self._gripper_right_cli, self._gripper_right,
                )
                self._gripper_right = None

    def _send_gripper_goal(self, client, position):
        """发送 GripperCommand action goal。"""
        if not client.wait_for_server(timeout_sec=0.0):
            return
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 10.0  # 默认力矩限制
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(self._gripper_goal_response)

    @staticmethod
    def _gripper_goal_response(future):
        """处理 gripper goal 响应（仅日志）。"""
        try:
            gh = future.result()
            if not gh.accepted:
                pass  # gripper 经常被同时触发，静默忽略
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IoTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
