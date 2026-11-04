# -*- coding: utf-8 -*-
"""Демо: управлять кистью шестимерным СМЕЩЕНИЕМ (delta), TwistStamped.

Запуск:
    ros2 run arm_ik demo_delta_publisher --ros-args -p use_sim_time:=true \
        -p side:=right -p dx:=0.05 -p period:=6.0

Каждые `period` секунд публикует на /arm_ik/delta_{side} одну и ту же
дельту (linear = dxyz в метрах, angular = rotvec в радианах). При
профиле узла delta_reference=measured (по умолчанию) это «шагающий»
режим: каждая команда смещает кисть ещё на dx от её фактического
положения. Для идемпотентной дельты относительно последней цели —
запустите arm_ik_node с -p delta_reference:=last_goal.

Для разового теста из командной строки достаточно:
    ros2 topic pub -1 /arm_ik/delta_right geometry_msgs/msg/TwistStamped \
      "{header: {frame_id: pelvis}, twist: {linear: {x: 0.05}, angular: {z: 0.2}}}"
"""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class DemoDelta(Node):
    def __init__(self):
        super().__init__("demo_delta_publisher")
        self.declare_parameter("side", "right")          # left | right | both
        self.declare_parameter("dx", 0.05)               # м, вперёд (+x корпуса)
        self.declare_parameter("dy", 0.0)
        self.declare_parameter("dz", 0.0)
        self.declare_parameter("rx", 0.0)                # рад, rotvec-компоненты
        self.declare_parameter("ry", 0.0)
        self.declare_parameter("rz", 0.0)
        self.declare_parameter("period", 6.0)            # с между дельтами
        side = str(self.get_parameter("side").value)
        topics = {"/arm_ik/delta_left", "/arm_ik/delta_right"} \
            if side == "both" else {f"/arm_ik/delta_{side}"}
        self.pubs = [self.create_publisher(TwistStamped, t, 10) for t in sorted(topics)]
        self.create_timer(float(self.get_parameter("period").value), self.tick)
        self.get_logger().info(f"demo delta -> {sorted(topics)}")

    def tick(self):
        g = lambda n: float(self.get_parameter(n).value)
        ts = TwistStamped()
        ts.header.frame_id = "pelvis"
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist.linear.x, ts.twist.linear.y, ts.twist.linear.z = g("dx"), g("dy"), g("dz")
        ts.twist.angular.x, ts.twist.angular.y, ts.twist.angular.z = g("rx"), g("ry"), g("rz")
        for p in self.pubs:
            p.publish(ts)
        self.get_logger().info(
            f"delta dp=({ts.twist.linear.x:+.3f},{ts.twist.linear.y:+.3f},"
            f"{ts.twist.linear.z:+.3f}) dr=({ts.twist.angular.x:+.3f},"
            f"{ts.twist.angular.y:+.3f},{ts.twist.angular.z:+.3f})")


def main():
    rclpy.init()
    node = DemoDelta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
