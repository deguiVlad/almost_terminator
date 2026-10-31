# Starter node: reads joints, the head camera and the task, turns the head and opens/closes the right hand.
# Author: Nikolay Semenov <nssr.nikolay@gmail.com>
import json
import math

import rclpy
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

HEAD_YAW = 3


class Hello(Node):
    def __init__(self):
        super().__init__("hello_world")
        self.joints = {}
        self.image = None
        self.depth_mm = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Image, "/cam_head/color/image_raw", self.on_image, 2)
        self.create_subscription(Image, "/cam_head/depth/image_raw", self.on_depth, 2)
        self.create_subscription(String, "/sim/task", self.on_task, latched)
        self.head = self.create_publisher(CmdSetMotorPosition, "/head/cmd_pos", 10)
        self.hand = self.create_publisher(JointState, "/inspire_hand/ctrl/right_hand", 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.1, self.step)
        self.create_timer(2.0, self.report)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_image(self, msg):
        self.image = (msg.width, msg.height, msg.encoding)

    def on_depth(self, msg):
        x, y = msg.width // 2, msg.height // 2
        i = (y * msg.width + x) * 2
        self.depth_mm = int.from_bytes(bytes(msg.data[i:i + 2]), "little")

    def on_task(self, msg):
        self.get_logger().info("task: " + json.dumps(json.loads(msg.data), ensure_ascii=False))

    def step(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        cmd = CmdSetMotorPosition()
        cmd.cmds = [SetMotorPosition(name=HEAD_YAW, pos=0.5 * math.sin(0.5 * t), spd=1.0, cur=1.0)]
        self.head.publish(cmd)
        hand = JointState()
        hand.name = [str(i) for i in range(1, 7)]
        closed = int(t / 3.0) % 2 == 1
        hand.position = [0.2, 0.2, 0.2, 0.2, 0.5, 0.2] if closed else [1.0] * 6
        self.hand.publish(hand)

    def report(self):
        yaw = self.joints.get("head_yaw_joint")
        self.get_logger().info(
            f"head yaw {yaw if yaw is None else round(yaw, 3)} rad | head camera {self.image} | "
            f"depth at image centre {self.depth_mm} mm")


def main():
    rclpy.init()
    node = Hello()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
