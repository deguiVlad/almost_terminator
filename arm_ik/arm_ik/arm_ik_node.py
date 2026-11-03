# -*- coding: utf-8 -*-
"""ROS 2 сервис-нода IK для обеих рук TienKung (7 DOF на руку).

Интерфейсы:
  * сервис ~/solve (arm_ik/srv/SolveIK): цель PoseStamped в рамке pelvis + side
    -> решённые углы суставов (для отладки/планирования, без движения);
  * подписка /arm_ik/goal_left, /arm_ik/goal_right (geometry_msgs/PoseStamped):
    полученная цель сразу исполняется — IK + траектория + cmd_pos с замыканием
    контура по /joint_states (недолёт ПД-приводов до 15 % компенсируется);
  * публикация /arm_ik/wrist_{left,right}_pose (PoseStamped) — текущая поза TCP
    по FK от измеренных суставов (полезно для RViz и верхнего уровня).

Коллизия со столом: TableGuard (полуплоскость z=h_table в прямоугольнике стола),
мягкий барьер в IK + проверка каждой точки траектории; при конфликте — повторный
заход с «вершинным» waypoint над столом.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

from arm_ik.kinematics import (ArmModel, fk, pose_error, ik_dls, clip_to_limits,
                               load_arm_model)
from arm_ik.collision import TableGuard
from arm_ik.trajectory import joint_space_profile
from arm_ik.srv import SolveIK


def pose_to_T(pose_stamped_msg_in_base):
    p = pose_stamped_msg_in_base.pose.position
    q = pose_stamped_msg_in_base.pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class ArmIKNode(Node):
    def __init__(self):
        super().__init__("arm_ik_node")
        # --- параметры ---
        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("model_path", "")          # config/arms.json (из URDF)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("spd_limit", 0.8)          # жёсткий лимит документации
        self.declare_parameter("a_max", 0.4)
        self.declare_parameter("tol_pos", 0.01)           # м
        self.declare_parameter("tol_rot", 0.05)           # рад
        self.declare_parameter("kp", 0.6)                 # коррекция недолёта ПД
        self.declare_parameter("h_table", 0.9)            # высота стола над pelvis, м
        self.declare_parameter("table_x0", 0.35)
        self.declare_parameter("table_x1", 1.2)
        self.declare_parameter("table_y_half", 1.0)
        self.declare_parameter("table_margin", 0.02)
        self.declare_parameter("enable_table_guard", True)

        mp = self.get_parameter("model_path").value or None
        self.models = {
            "left": load_arm_model(mp, "left"),
            "right": load_arm_model(mp, "right"),
        }
        tg = TableGuard(self.get_parameter("h_table").value,
                        self.get_parameter("table_x0").value,
                        self.get_parameter("table_x1").value,
                        self.get_parameter("table_y_half").value,
                        self.get_parameter("table_margin").value) \
            if self.get_parameter("enable_table_guard").value else None
        self.guard = tg

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.spd = float(self.get_parameter("spd_limit").value)
        self.a_max = float(self.get_parameter("a_max").value)
        self.tol_pos = float(self.get_parameter("tol_pos").value)
        self.tol_rot = float(self.get_parameter("tol_rot").value)
        self.kp = float(self.get_parameter("kp").value)

        # --- состояние ---
        self.joints = {}
        self.exec_ = {}   # side -> dict(traj=..., idx=..., T_des=..., done=cb)
        for s in ("left", "right"):
            q0 = np.zeros(7)
            self.exec_[s] = {"traj": None, "idx": 0, "T_des": np.eye(4),
                             "q_hold": q0.copy()}

        # --- ROS интерфейсы ---
        self.cmd_pub = self.create_publisher(CmdSetMotorPosition, "/arm/cmd_pos", 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(PoseStamped, "/arm_ik/goal_left",
                                 lambda m: self.on_goal("left", m), 10)
        self.create_subscription(PoseStamped, "/arm_ik/goal_right",
                                 lambda m: self.on_goal("right", m), 10)
        self.pose_pubs = {
            "left": self.create_publisher(PoseStamped, "/arm_ik/wrist_left_pose", 10),
            "right": self.create_publisher(PoseStamped, "/arm_ik/wrist_right_pose", 10),
        }
        cbg = ReentrantCallbackGroup()
        self.srv = self.create_service(SolveIK, "~/solve", self.solve_srv,
                                       callback_group=cbg)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)
        self.get_logger().info(
            f"arm_ik_node ready | models: {[m.tip_link for m in self.models.values()]} | "
            f"table guard {'ON' if tg else 'OFF'} h={tg.h if tg else '-'}")

    # ------------------------------------------------------------------ helpers
    def on_js(self, msg: JointState):
        self.joints.update(dict(zip(msg.name, msg.position)))

    def q_meas(self, model: ArmModel):
        return np.array([self.joints.get(n, 0.0) for n in model.names])

    def make_cmd(self, model: ArmModel, q_target, spd=None):
        msg = CmdSetMotorPosition()
        spd = self.spd if spd is None else spd
        msg.cmds = [SetMotorPosition(name=int(mid), pos=float(q), spd=float(spd), cur=1.0)
                    for mid, q in zip(model.ids, q_target)]
        return msg

    # ------------------------------------------------------------------ goals
    def on_goal(self, side: str, goal: PoseStamped):
        model = self.models[side]
        if goal.header.frame_id not in ("", "pelvis", "base_link", model.base_frame):
            self.get_logger().warn(
                f"[{side}] цель в рамке {goal.header.frame_id!r} — узел ожидает pelvis; "
                f"перенесите через TF перед публикацией")
        T_des = pose_to_T(goal)
        # жёсткая страховка цели от стола
        if self.guard is not None:
            T_des[:3, 3] = self.guard.clamp_target_above_table(T_des[:3, 3])
        st = self.exec_[side]
        st["T_des"] = T_des
        q_meas = self.q_meas(model)
        # 1) IK к цели из текущей измеренной позы
        q_sol, ok, _ = ik_dls(model, T_des, q_init=q_meas, max_iter=200,
                              tol_pos=self.tol_pos, tol_rot=self.tol_rot)
        if not ok and self.guard is not None:
            # 2) заход через вершину: waypoint над столом
            wp = T_des.copy()
            wp[:3, 3] = self.guard.clamp_target_above_table(wp[:3, 3] + np.array([0, 0, 0.25]))
            q_wp, ok1, _ = ik_dls(model, wp, q_init=q_meas, max_iter=150)
            q_sol2, ok2, _ = ik_dls(model, T_des, q_init=q_wp, max_iter=150)
            if ok1 and ok2:
                traj1 = joint_space_profile(q_meas, q_wp, self.spd, self.a_max, 1.0 / self.rate_hz)
                traj2 = joint_space_profile(q_wp, q_sol2, self.spd, self.a_max, 1.0 / self.rate_hz)
                st["traj"] = np.vstack([traj1[:-1], traj2])
                st["idx"] = 0
                self.get_logger().info(f"[{side}] IK через waypoint над столом")
                return
        if not ok:
            self.get_logger().error(f"[{side}] IK не сошлась к цели — удерживаем текущую позу")
            st["traj"] = None
            return
        # фильтрация траектории от коллизий со столом
        if self.guard is not None:
            bad = [i for i, qq in enumerate(joint_space_profile(q_meas, q_sol, self.spd, self.a_max, 1.0 / self.rate_hz))
                   if self.guard.collides_with_table(model, qq)]
            if bad:
                self.get_logger().warn(f"[{side}] траектория пересекает стол ({len(bad)} точек) — стоп")
                st["traj"] = None
                return
        st["traj"] = joint_space_profile(q_meas, q_sol, self.spd, self.a_max, 1.0 / self.rate_hz)
        st["idx"] = 0
        self.get_logger().info(
            f"[{side}] IK решена: {len(st['traj'])} точек, ~{(len(st['traj'])-1)/self.rate_hz:.1f} c")

    # ------------------------------------------------------------------ service
    def solve_srv(self, req, res):
        side = req.side if req.side in ("left", "right") else "right"
        model = self.models[side]
        T_des = pose_to_T(req.goal)
        q_init = self.q_meas(model) if not req.use_zero_seed else np.zeros(7)
        q, ok, hist = ik_dls(model, T_des, q_init, max_iter=200)
        res.success = bool(ok)
        res.joint_names = model.names
        res.positions = [float(v) for v in q]
        res.motor_ids = [int(i) for i in model.ids]
        res.final_error = float(hist[-1]) if hist else 1e9
        return res

    # ------------------------------------------------------------------ control loop
    def step(self):
        now = self.get_clock().now().to_msg()
        for side, model in self.models.items():
            st = self.exec_[side]
            q_meas = self.q_meas(model)
            # публикация текущей позы TCP (FK от энкодеров)
            ps = PoseStamped()
            ps.header.frame_id = model.base_frame
            ps.header.stamp = now
            T, _ = fk(model, q_meas)
            Tcp = T.copy()
            from arm_ik.kinematics import _tmat
            Tcp = Tcp @ _tmat(model.tcp_offset, [0, 0, 0])
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = Tcp[:3, 3]
            qx, qy, qz, qw = Rotation.from_matrix(Tcp[:3, :3]).as_quat()
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            self.pose_pubs[side].publish(ps)

            if st["traj"] is None:
                continue
            i = st["idx"]
            if i >= len(st["traj"]):
                # фаза точного позиционирования: P-коррекция недолёта ПД-приводов
                e = pose_error(fk(model, q_meas)[0], st["T_des"])
                if np.linalg.norm(e[:3]) < self.tol_pos and np.linalg.norm(e[3:]) < self.tol_rot:
                    st["traj"] = None
                    self.get_logger().info(f"[{side}] цель достигнута (pos err {np.linalg.norm(e[:3])*100:.1f} см)")
                    continue
                from arm_ik.kinematics import geometric_jacobian
                J = geometric_jacobian(model, q_meas)
                A = J @ J.T + 0.05 ** 2 * np.eye(6)
                dq = J.T @ np.linalg.solve(A, e)
                dq = np.clip(dq, -0.1, 0.1)  # ≤ 0.1 рад/цикл как в FOCUS_TRACKING.md §4
                q_new = clip_to_limits(q_meas + self.kp * dq, model, margin=1e-4)
                st["traj"] = joint_space_profile(q_meas, q_new, min(self.spd, 0.3),
                                                 self.a_max, 1.0 / self.rate_hz)
                st["idx"] = 0
                continue
            q_ref = st["traj"][i]
            self.cmd_pub.publish(self.make_cmd(model, q_ref))
            st["idx"] += 1


def main():
    rclpy.init()
    node = ArmIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
