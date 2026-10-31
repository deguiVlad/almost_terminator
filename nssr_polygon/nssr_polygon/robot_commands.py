# Robot command topics and services with the semantics of the real robot (speeds, velocity mode, hands),
# hand state and wrist force/torque.
# Author: Nikolay Semenov <nssr.nikolay@gmail.com>
import math, os, queue, threading, time
import xml.etree.ElementTree as ET

MOTORS = {1: "head_roll_joint", 2: "head_pitch_joint", 3: "head_yaw_joint", 31: "body_yaw_joint",
          11: "shoulder_pitch_l_joint", 12: "shoulder_roll_l_joint", 13: "shoulder_yaw_l_joint",
          14: "elbow_pitch_l_joint", 15: "elbow_yaw_l_joint", 16: "wrist_pitch_l_joint", 17: "wrist_roll_l_joint",
          21: "shoulder_pitch_r_joint", 22: "shoulder_roll_r_joint", 23: "shoulder_yaw_r_joint",
          24: "elbow_pitch_r_joint", 25: "elbow_yaw_r_joint", 26: "wrist_pitch_r_joint", 27: "wrist_roll_r_joint"}
GROUPS = {"arm": [11, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26, 27], "head": [1, 2, 3], "waist": [31]}
HAND = {"left": ["left_little_1_joint", "left_ring_1_joint", "left_middle_1_joint", "left_index_1_joint",
                 "left_thumb_2_joint", "left_thumb_1_joint"],
        "right": ["right_little_1_joint", "right_ring_1_joint", "right_middle_1_joint", "right_index_1_joint",
                  "right_thumb_2_joint", "right_thumb_1_joint"]}
FULL_STROKE_S = 0.8
VEL_TIMEOUT = 0.25
INF = float("inf")
_MS = float(os.environ.get("HACK_MAX_JOINT_SPEED", "0.8"))
MAX_SPEED = _MS if _MS > 0 else INF


URDF_REL = "assets/robots/tiangong_pro/tiangong_pro_with_hands.urdf"


def find_urdf():
    if os.environ.get("HACK_URDF"):
        return os.environ["HACK_URDF"]
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        if os.path.isfile(os.path.join(d, URDF_REL)):
            return os.path.join(d, URDF_REL)
        d = os.path.dirname(d)
    return os.path.join("/nssr_sim", URDF_REL)


def _limits(urdf):
    lim = {}
    try:
        for j in ET.parse(urdf).getroot().findall("joint"):
            l = j.find("limit")
            if l is not None and j.get("type") == "revolute":
                lim[j.get("name")] = (float(l.get("lower")), float(l.get("upper")))
    except Exception:
        pass
    return lim


class Shaper:
    def __init__(self, node, send):
        self.n, self.send = node, send
        self.lock = threading.Lock()
        urdf = find_urdf()
        self.lim = _limits(urdf)
        self.j = {}
        self.f = {s: {"tgt": [None] * 6, "goal": [None] * 6, "speed": [1.0] * 6} for s in HAND}
        self.meas, self.pct, self.sim_t, self.loop_t = {}, {}, None, None
        self.prev_pct = {}
        self.effort, self.ft, self.fvel = {}, None, {s: [0.0] * 6 for s in HAND}
        self._mk_ros()
        threading.Thread(target=self._loop, daemon=True).start()
        node.get_logger().info("hack shaper: cmd_pos spd, cmd_vel, hand speed/force/state, wrist F/T")

    def _mk_ros(self):
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import WrenchStamped
        from bodyctrl_msgs.msg import CmdSetMotorPosition, CmdMotorCtrl, CmdSetMotorSpeed
        n = self.n
        self.JS, self.WS = JointState, WrenchStamped
        for g in GROUPS:
            n.create_subscription(CmdSetMotorPosition, f"/{g}/cmd_pos", lambda m: self.on_pos(m, True), 10)
            n.create_subscription(CmdMotorCtrl, f"/{g}/cmd_ctrl", lambda m: self.on_pos(m, False), 10)
            n.create_subscription(CmdSetMotorSpeed, f"/{g}/cmd_vel", self.on_vel, 10)
        self.pub_state, self.pub_ft = {}, {}
        for s in HAND:
            n.create_subscription(JointState, f"/inspire_hand/ctrl/{s}_hand", lambda m, s=s: self.on_hand(m, s), 10)
            self.pub_state[s] = n.create_publisher(JointState, f"/inspire_hand/state/{s}_hand", 10)
            self.pub_ft[s] = n.create_publisher(WrenchStamped, f"/arm_6dof_{s}", 10)
        try:
            from bodyctrl_msgs.srv import SetAngleFlexible, SetSpeed, SetForce, SetClearError
            for s in HAND:
                n.create_service(SetAngleFlexible, f"/inspire_hand/set_angle_flexible/{s}_hand",
                                 lambda rq, rs, s=s: self.srv_angle(rq, rs, s))
                n.create_service(SetSpeed, f"/inspire_hand/set_speed/{s}_hand", lambda rq, rs, s=s: self.srv_speed(rq, rs, s))
                n.create_service(SetForce, f"/inspire_hand/set_force/{s}_hand", lambda rq, rs, s=s: self.srv_force(rq, rs, s))
                n.create_service(SetClearError, f"/inspire_hand/set_clear_error/{s}_hand", self.srv_clear)
        except Exception as e:
            n.get_logger().error(f"hand services not available (bodyctrl_msgs.srv): {e}")

    def _clamp(self, name, x):
        lo, hi = self.lim.get(name, (-INF, INF))
        return min(hi, max(lo, x))

    def _joint(self, name):
        st = self.j.get(name)
        if st is None:
            st = self.j[name] = {"tgt": self.meas.get(name), "goal": None, "vmax": INF, "mode": "pos", "v": 0.0,
                                 "t_vel": 0.0}
        return st

    def on_pos(self, m, use_spd):
        with self.lock:
            for c in m.cmds:
                name = MOTORS.get(int(c.name))
                if name is None:
                    continue
                st = self._joint(name)
                st["mode"], st["goal"] = "pos", self._clamp(name, float(c.pos))
                spd = float(c.spd) if use_spd else 0.0
                st["vmax"] = min(abs(spd), MAX_SPEED) if spd > 0 else MAX_SPEED

    def on_vel(self, m):
        now = time.time()
        with self.lock:
            for c in m.cmds:
                name = MOTORS.get(int(c.name))
                if name is None:
                    continue
                st = self._joint(name)
                st["mode"], st["v"], st["t_vel"] = "vel", max(-MAX_SPEED, min(MAX_SPEED, float(c.spd))), now

    def _set_fingers(self, side, ids, vals):
        for i, v in zip(ids, vals):
            if 1 <= i <= 6:
                self.f[side]["goal"][i - 1] = min(1.0, max(0.0, float(v)))

    def on_hand(self, m, side):
        ids = []
        for nm in m.name:
            try:
                ids.append(int(nm))
            except ValueError:
                ids.append(-1)
        with self.lock:
            self._set_fingers(side, ids, m.position)

    def srv_angle(self, rq, rs, side):
        ids = []
        for nm in rq.name:
            try:
                ids.append(int(nm))
            except ValueError:
                ids.append(-1)
        with self.lock:
            self._set_fingers(side, ids, rq.angle_ratio)
        rs.angle_accepted = True
        return rs

    def srv_speed(self, rq, rs, side):
        v = [rq.speed0_ratio, rq.speed1_ratio, rq.speed2_ratio, rq.speed3_ratio, rq.speed4_ratio, rq.speed5_ratio]
        with self.lock:
            self.f[side]["speed"] = [min(1.0, max(0.01, float(x))) for x in v]
        rs.speed_accepted = True
        return rs

    def srv_force(self, rq, rs, side):
        v = [rq.force0_ratio, rq.force1_ratio, rq.force2_ratio, rq.force3_ratio, rq.force4_ratio, rq.force5_ratio]
        self.send({"hack_hand_force": {side: [min(1.0, max(0.0, float(x))) for x in v]}})
        rs.force_accepted = True
        return rs

    def srv_clear(self, rq, rs):
        rs.setclear_error_accepted = True
        return rs

    def on_status(self, st):
        t = st.get("sim_time")
        if "joint_names" in st:
            meas = dict(zip(st["joint_names"], st["joint_pos"]))
        else:
            meas = None
        pct = st.get("finger_percentages") or {}
        with self.lock:
            if t is not None and self.sim_t is not None and t < self.sim_t - 0.5:
                self.j.clear()
                for s in HAND:
                    self.f[s]["tgt"], self.f[s]["goal"] = [None] * 6, [None] * 6
                self.loop_t = None
                self._reset = True
            if meas is not None:
                self.meas = meas
            if pct and t is not None:
                for s, names in HAND.items():
                    for k, nm in enumerate(names):
                        if nm in pct:
                            prev = self.prev_pct.get(nm)
                            if prev is not None and t > prev[1]:
                                rate = (pct[nm] - prev[0]) / (t - prev[1])
                                self.fvel[s][k] = max(-1.0, min(1.0, rate * FULL_STROKE_S))
                            self.prev_pct[nm] = (pct[nm], t)
                self.pct = pct
            if t is not None:
                self.sim_t = t
            self.effort = st.get("hack_finger_effort") or {}
            self.ft = st.get("hack_ft")
        self._publish(t)

    def _publish(self, t):
        if t is None:
            return
        from builtin_interfaces.msg import Time
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9) % 1_000_000_000)
        for s, names in HAND.items():
            m = self.JS()
            m.header.stamp = stamp
            m.name = [str(i) for i in range(1, 7)]
            m.position = [float(self.pct.get(nm, 0.0)) for nm in names]
            m.velocity = [float(x) for x in self.fvel[s]]
            m.effort = [float(x) for x in (self.effort.get(s) or [0.0] * 6)]
            self.pub_state[s].publish(m)
        if self.ft:
            for s, v in self.ft.items():
                if s not in self.pub_ft:
                    continue
                w = self.WS()
                w.header.stamp = stamp
                w.header.frame_id = f"wrist_roll_{s[0]}_link"
                w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = (float(x) for x in v[:3])
                w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = (float(x) for x in v[3:6])
                self.pub_ft[s].publish(w)

    def _loop(self):
        self._sent = {}
        while True:
            time.sleep(0.005)
            ch = self.step(time.time())
            if ch:
                self.send(ch)

    def step(self, now):
        sent = self.__dict__.setdefault("_sent", {})
        out = {}
        with self.lock:
            if getattr(self, "_reset", False):
                sent.clear()
                self._reset = False
            if self.sim_t is None:
                return {}
            dt = 0.0 if self.loop_t is None else max(0.0, self.sim_t - self.loop_t)
            self.loop_t = self.sim_t
            for name, st in self.j.items():
                if st["tgt"] is None:
                    st["tgt"] = self.meas.get(name, st["goal"] if st["goal"] is not None else 0.0)
                if st["mode"] == "vel":
                    v = st["v"] if now - st["t_vel"] < VEL_TIMEOUT else 0.0
                    st["tgt"] = self._clamp(name, st["tgt"] + v * dt)
                elif st["goal"] is not None:
                    e = st["goal"] - st["tgt"]
                    step = st["vmax"] * dt if st["vmax"] < INF else INF
                    st["tgt"] = st["goal"] if abs(e) <= step else st["tgt"] + math.copysign(step, e)
                out[name] = st["tgt"]
            for s, names in HAND.items():
                f = self.f[s]
                for k, nm in enumerate(names):
                    if f["goal"][k] is None:
                        continue
                    if f["tgt"][k] is None:
                        f["tgt"][k] = float(self.pct.get(nm, f["goal"][k]))
                    e = f["goal"][k] - f["tgt"][k]
                    step = f["speed"][k] / FULL_STROKE_S * dt
                    f["tgt"][k] = f["goal"][k] if abs(e) <= step else f["tgt"][k] + math.copysign(step, e)
                    out[nm] = f["tgt"][k]
        ch = {k: v for k, v in out.items() if abs(sent.get(k, 1e9) - v) > 1e-6}
        sent.update(ch)
        return ch
