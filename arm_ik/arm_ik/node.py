"""IK-нода обеих рук: цель PoseStamped -> углы суставов -> /arm/cmd_pos с дотягиванием по /joint_states.

Вход:  /robot_description (URDF, latched), /joint_states,
       /arm_ik/goal_{left,right} (PoseStamped; рамка любая из TF, пусто = pelvis).
Выход: /arm/cmd_pos, /arm_ik/{left,right}/status (JSON), /arm_ik/wrist_{left,right}_pose.
"""
import json

import numpy as np
import rclpy
import tf2_ros
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_ik.kinematics import Arm, R_to_quat, make_T, quat_to_R, rot_error

SIDES = ("left", "right")


def pose_to_T(p):
    return make_T(quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                  [p.position.x, p.position.y, p.position.z])


class ArmIK(Node):
    def __init__(self):
        super().__init__("arm_ik")
        self.base = self.declare_parameter("base_frame", "pelvis").value
        self.tips = {s: self.declare_parameter(f"tip_link_{s}", "").value or None for s in SIDES}
        self.tcp = list(self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0]).value)
        self.spd = min(0.8, self.declare_parameter("spd", 0.8).value)
        self.rot_weight = self.declare_parameter("rot_weight", 1.0).value   # 0 = только позиция
        self.tol_pos = self.declare_parameter("tol_pos", 0.01).value
        self.tol_rot = self.declare_parameter("tol_rot", 0.05).value
        self.kp = self.declare_parameter("kp", 0.7).value                   # доля ошибки на коррекцию
        self.joint_tol = self.declare_parameter("joint_tol", 0.005).value
        self.max_corr = self.declare_parameter("max_corrections", 15).value
        self.rate = self.declare_parameter("rate_hz", 20.0).value

        self.arms = {}
        self.joints = {}
        self.task = {s: {"state": "idle"} for s in SIDES}
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self.on_urdf, latched)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        for s in SIDES:
            self.create_subscription(PoseStamped, f"/arm_ik/goal_{s}", lambda m, s=s: self.on_goal(s, m), 10)
        self.cmd = self.create_publisher(CmdSetMotorPosition, "/arm/cmd_pos", 10)
        self.status = {s: self.create_publisher(String, f"/arm_ik/{s}/status", 10) for s in SIDES}
        self.wrist = {s: self.create_publisher(PoseStamped, f"/arm_ik/wrist_{s}_pose", 10) for s in SIDES}
        self.create_timer(1.0 / self.rate, self.tick)

    def on_urdf(self, msg):
        try:
            self.arms = {s: Arm(msg.data, s, self.base, self.tips[s], self.tcp) for s in SIDES}
        except ValueError as e:
            self.get_logger().error(f"URDF: {e}")
            return
        self.get_logger().info("модель рук из /robot_description: "
                               + ", ".join(f"{s}: {a.base} -> {a.tip}" for s, a in self.arms.items()))

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def q(self, arm):
        return np.array([self.joints.get(n, 0.0) for n in arm.names])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, arm, q, spd):
        msg = CmdSetMotorPosition()
        # spd: 0 в симуляторе значит «предельная», поэтому снизу ограничиваем
        msg.cmds = [SetMotorPosition(name=i, pos=float(v), spd=float(max(0.02, s)), cur=1.0)
                    for i, v, s in zip(arm.motor_ids, q, np.broadcast_to(spd, len(q)))]
        self.cmd.publish(msg)

    def on_goal(self, side, msg):
        arm = self.arms.get(side)
        if arm is None:
            self.get_logger().warn("нет /robot_description — цель пропущена")
            return
        T = pose_to_T(msg.pose)
        frame = msg.header.frame_id
        if frame and frame != self.base:
            try:
                t = self.tf.lookup_transform(self.base, frame, Time(), timeout=Duration(seconds=0.5))
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f"нет TF {self.base} <- {frame}: {e}")
                return
            r, p = t.transform.rotation, t.transform.translation
            T = make_T(quat_to_R(r.x, r.y, r.z, r.w), [p.x, p.y, p.z]) @ T

        q_now = self.q(arm)
        ok, q_goal, ep, er = arm.solve(T, q_now, self.joints, tol_pos=self.tol_pos / 2,
                                       tol_rot=self.tol_rot / 2, rot_weight=self.rot_weight)
        if not ok:
            self.task[side] = {"state": "unreachable", "err_pos": ep, "err_rot": er}
            self.get_logger().warn(f"[{side}] цель недостижима: {ep * 100:.1f} см, {er:.2f} рад")
            return
        dq = np.abs(q_goal - q_now)
        dmax = max(float(dq.max()), 1e-6)
        self.send(arm, q_goal, self.spd * dq / dmax)    # все суставы приходят одновременно
        self.task[side] = {"state": "moving", "T": T, "q_goal": q_goal, "q_cmd": q_goal.copy(),
                           "settle_at": self.now() + dmax / self.spd, "q_prev": q_now, "still": 0, "corr": 0}

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for side, arm in self.arms.items():
            q = self.q(arm)
            T_meas = arm.fk(q, self.joints)
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = self.base, stamp
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = T_meas[:3, 3]
            o = ps.pose.orientation
            o.x, o.y, o.z, o.w = R_to_quat(T_meas[:3, :3])
            self.wrist[side].publish(ps)

            t = self.task[side]
            if t["state"] == "moving":
                # остановилась = 3 тика подряд почти без движения (один тик мог прийти без новых /joint_states)
                still = np.max(np.abs(q - t["q_prev"])) * self.rate < 0.02
                t["still"] = t["still"] + 1 if still else 0
                t["q_prev"] = q
                if t["still"] >= 3 and self.now() >= t["settle_at"]:
                    e = t["q_goal"] - q
                    if np.max(np.abs(e)) < self.joint_tol or t["corr"] >= self.max_corr:
                        ep = float(np.linalg.norm(t["T"][:3, 3] - T_meas[:3, 3]))
                        er = float(np.linalg.norm(rot_error(T_meas[:3, :3], t["T"][:3, :3])))
                        hit = ep < self.tol_pos and (self.rot_weight == 0 or er < self.tol_rot)
                        self.task[side] = t = {"state": "reached" if hit else "stopped",
                                               "err_pos": ep, "err_rot": er}
                    else:
                        # ПД-приводы недотягивают под весом руки: сдвигаем команду на долю ошибки
                        t["q_cmd"] = np.clip(t["q_cmd"] + self.kp * e, arm.lower, arm.upper)
                        self.send(arm, t["q_cmd"], 0.3)
                        t["corr"] += 1
                        t["still"] = 0
                        t["settle_at"] = self.now() + 0.2
            st = {"state": t["state"]}
            st.update({k: round(t[k], 4) for k in ("err_pos", "err_rot") if k in t})
            self.status[side].publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = ArmIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
