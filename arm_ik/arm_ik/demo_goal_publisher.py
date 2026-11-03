# -*- coding: utf-8 -*-
"""Демо: опубликовать цели PoseStamped для обеих рук (рамка pelvis).

Запуск:
    ros2 run arm_ik demo_goal_publisher --ros-args -p use_sim_time:=true \
        -p x:=0.45 -p y_r:=-0.25 -p y_l:=0.25 -p z:=1.0

Посылает по одной цели на руку с ориентацией «кисть смотрит вниз» (заход сверху),
повторяет каждые 8 секунд. Используется для ручной проверки IK без верхнего уровня.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation


class DemoGoals(Node):
    def __init__(self):
        super().__init__("demo_goal_publisher")
        self.declare_parameter("x", 0.45)
        self.declare_parameter("y_l", 0.25)
        self.declare_parameter("y_r", -0.25)
        self.declare_parameter("z", 1.0)
        self.declare_parameter("yaw_deg", -90.0)
        self.pub_l = self.create_publisher(PoseStamped, "/arm_ik/goal_left", 10)
        self.pub_r = self.create_publisher(PoseStamped, "/arm_ik/goal_right", 10)
        self.create_timer(8.0, self.tick)
        self.get_logger().info("demo goals: pelvis x,y,z + yaw; repeat every 8 s")

    def make(self, x, y, z, yaw):
        ps = PoseStamped()
        ps.header.frame_id = "pelvis"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        q = Rotation.from_euler("xyz", [math.radians(yaw), 0.0, 0.0]).as_quat()
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        return ps

    def tick(self):
        g = lambda: (float(self.get_parameter("x").value), float(self.get_parameter("z").value))
        x, z = g()
        yaw = float(self.get_parameter("yaw_deg").value)
        yl = float(self.get_parameter("y_l").value)
        yr = float(self.get_parameter("y_r").value)
        self.pub_l.publish(self.make(x, yl, z, yaw))
        self.pub_r.publish(self.make(x, yr, z, yaw))
        self.get_logger().info(f"published goals L({x},{yl},{z}) R({x},{yr},{z}) yaw={yaw}")


def main():
    rclpy.init()
    node = DemoGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
