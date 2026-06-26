#!/usr/bin/env python3
"""
Pad 桥接节点 — 订阅 /pad_control，拆分并发布到各执行话题。
当前: L1/R1 → 左右夹爪控制
可扩展: 摇杆 → AGV 速度, 按钮 → 其他指令
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class TeleopPadBridge(Node):

    def __init__(self):
        super().__init__("teleop_pad_bridge")

        self.declare_parameter("gripper_open", -0.98)
        self.declare_parameter("gripper_closed", 0.0)
        
        self.create_subscription(Joy, "/pico4_joy", self.pico4_joy_callback, 10)
        self.left_grip_pub = self.create_publisher(Float64MultiArray, "/teleop_pad/left/gripper_command", 10)
        self.right_grip_pub = self.create_publisher(Float64MultiArray, "/teleop_pad/right/gripper_command", 10)

        self.cmd_ctl_left = self.create_publisher(Float64MultiArray, "/cmd_ctl_left", 10)
        self.cmd_ctl_right = self.create_publisher(Float64MultiArray, "/cmd_ctl_right", 10)

        self.cmd_ctl_left_button = 0
        self.cmd_ctl_right_button = 0

        self.get_logger().info("teleopPadBridge 已启动...")

    def pico4_joy_callback(self, joy_msg:Joy):
        # print(f"{msg.axes[2]} {msg.axes[6]}")
        self.gripper_control_pub(joy_msg.axes[3],joy_msg.axes[7])

        if  joy_msg.buttons[0]:
            if self.cmd_ctl_left_button == 0:
                msg = Float64MultiArray()
                msg.data = [1.0]
                self.cmd_ctl_left_button = 1
                self.cmd_ctl_left.publish(msg)
        else:
            self.cmd_ctl_left_button = 0

        if  joy_msg.buttons[6]:
            if self.cmd_ctl_right_button == 0:
                msg = Float64MultiArray()
                msg.data = [1.0]
                self.cmd_ctl_right_button = 1
                self.cmd_ctl_right.publish(msg)
        else:
            self.cmd_ctl_right_button = 0


    def gripper_control_pub(self, left_gripper_value, right_gripper_value):
        #左右夹爪控制转发
        open_value = self.get_parameter("gripper_open").value
        close_value = self.get_parameter("gripper_closed").value

        left_gripper_pos = open_value + (close_value - open_value) * left_gripper_value
        msg = Float64MultiArray()
        msg.data = [float(left_gripper_pos)]
        self.left_grip_pub.publish(msg)

        right_gripper_pos = open_value + (close_value - open_value) * right_gripper_value
        msg = Float64MultiArray()
        msg.data = [float(right_gripper_pos)]
        self.right_grip_pub.publish(msg)

    # def agv_control_pub(self,m: TeleopPadControl):
    #     pass


def main():
    rclpy.init()
    node = TeleopPadBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
