"""Кинематика 7-DOF руки TienKung 2.0 Pro по URDF (строка из /robot_description).

Цепочка строится от base (pelvis) до tip (wrist_roll_{l,r}_link). Суставы рук —
переменные IK; остальные подвижные суставы цепочки (body_yaw_joint) берутся как
есть из `extra` (обычно из /joint_states). Не зависит от ROS, только numpy.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw",
              "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll"]


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_R(axis, q):
    x, y, z = axis
    c, s = math.cos(q), math.sin(q)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """-> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def rot_error(R_cur, R_des):
    """Вектор поворота (ось * угол) от R_cur к R_des, в рамке базы."""
    E = R_des @ R_cur.T
    c = max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0))
    angle = math.acos(c)
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    if angle < 1e-6:
        return 0.5 * v
    if math.pi - angle < 1e-4:   # ~180°: E + I = 2·a·aᵀ, ось — самый длинный столбец
        M = E + np.eye(3)
        axis = M[:, int(np.argmax(np.linalg.norm(M, axis=0)))]
        return angle * axis / np.linalg.norm(axis)
    return angle * v / (2.0 * math.sin(angle))


def make_T(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def _floats(s, n=3):
    return [float(v) for v in s.split()] if s else [0.0] * n


class Arm:
    """Одна рука: цепочка суставов base -> tip из URDF."""

    def __init__(self, urdf_xml, side, base="pelvis", tip=None, tcp_offset=(0.0, 0.0, 0.0)):
        tag = "l" if side == "left" else "r"
        tip = tip or f"wrist_roll_{tag}_link"
        by_child = {}
        for j in ET.fromstring(urdf_xml).findall("joint"):
            o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
            by_child[j.find("child").get("link")] = {
                "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                "T": make_T(rpy_to_R(*_floats(o.get("rpy") if o is not None else None)),
                            _floats(o.get("xyz") if o is not None else None)),
                "axis": np.array(_floats(a.get("xyz") if a is not None else "1 0 0")),
                "lower": float(lim.get("lower", -math.pi)) if lim is not None else -math.pi,
                "upper": float(lim.get("upper", math.pi)) if lim is not None else math.pi}

        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"в URDF нет цепочки {base} -> {tip}")
            j = by_child[link]
            if j["type"] == "prismatic":
                raise ValueError(f"призматический сустав {j['name']} не поддерживается")
            chain.append(j)
            link = j["parent"]
        self.chain = chain[::-1]

        arm = [j for j in self.chain if j["type"] in ("revolute", "continuous")
               and j["name"].replace(f"_{tag}_joint", "") in ARM_JOINTS]
        if len(arm) != 7:
            raise ValueError(f"ожидалось 7 суставов руки, найдено {[j['name'] for j in arm]}")
        for j in arm:
            if j["type"] == "continuous":
                j["lower"], j["upper"] = -math.pi, math.pi
        self.side, self.base, self.tip = side, base, tip
        self.names = [j["name"] for j in arm]
        self.motor_ids = [(11 if side == "left" else 21) + ARM_JOINTS.index(n.replace(f"_{tag}_joint", ""))
                          for n in self.names]
        self.lower = np.array([j["lower"] for j in arm])
        self.upper = np.array([j["upper"] for j in arm])
        self._index = {n: i for i, n in enumerate(self.names)}
        self.tcp = make_T(p=np.array(tcp_offset, dtype=float))

    def _frames(self, q, extra):
        """Поза TCP и (ось, точка) каждого сустава руки в рамке base."""
        T = np.eye(4)
        axes = []
        for j in self.chain:
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                i = self._index.get(j["name"])
                angle = q[i] if i is not None else (extra or {}).get(j["name"], 0.0)
                if i is not None:
                    axes.append((T[:3, :3] @ j["axis"], T[:3, 3].copy()))
                T = T @ make_T(axis_angle_R(j["axis"], angle))
        return T @ self.tcp, axes

    def fk(self, q, extra=None):
        return self._frames(q, extra)[0]

    @staticmethod
    def _jac(T, axes):
        p = T[:3, 3]
        J = np.zeros((6, 7))
        for i, (z, o) in enumerate(axes):
            J[:3, i] = np.cross(z, p - o)
            J[3:, i] = z
        return J

    def jacobian(self, q, extra=None):
        return self._jac(*self._frames(q, extra))

    def wrist_lock(self):
        """Кисть соосна предплечью: wrist_pitch и wrist_roll держатся в нуле (прямая кисть в URDF)."""
        return {i: float(np.clip(0.0, self.lower[i], self.upper[i]))
                for i, n in enumerate(self.names) if n.startswith(("wrist_pitch", "wrist_roll"))}

    @staticmethod
    def _step(J, ep_v, er_v, orient, rot_weight, damping):
        I3 = damping ** 2 * np.eye(3)
        if orient == "full":
            w = np.array([1.0, 1.0, 1.0] + [rot_weight] * 3)
            Jw = J * w[:, None]
            return Jw.T @ np.linalg.solve(Jw @ Jw.T + damping ** 2 * np.eye(6), np.concatenate([ep_v, er_v]) * w)
        Jp = J[:3]
        Jp_pinv = Jp.T @ np.linalg.inv(Jp @ Jp.T + I3)
        dq = Jp_pinv @ ep_v
        if orient == "secondary":
            # ориентация — только движениями, которые не сдвигают позицию (нуль-пространство Jp)
            N = np.eye(J.shape[1]) - np.linalg.pinv(Jp, rcond=1e-4) @ Jp
            JrN = J[3:] @ N
            d2 = N @ (JrN.T @ np.linalg.solve(JrN @ JrN.T + I3, er_v - J[3:] @ dq))
            n = np.max(np.abs(d2))
            dq = dq + (d2 * (0.1 / n) if n > 0.1 else d2)   # вторичная задача не забивает шаг позиции
        return dq

    def ik(self, T_des, q0, extra=None, orient="full", rot_weight=1.0, locked=None, margin=0.05, iters=200,
           tol_pos=1e-3, tol_rot=1e-2, damping=0.05, max_step=0.2):
        """DLS от q0 в пределах суставов. -> (q, ошибка позиции м, ошибка ориентации рад).

        orient: "full" — вся поза (ориентация с весом rot_weight); "secondary" — сначала позиция,
        ориентация насколько позволяют оставшиеся степени свободы; "none" — только позиция.
        locked: {индекс сустава: угол} — суставы, которые не двигаются.
        margin: запас от пределов, рад — чтобы приводу оставался ход на дотягивание.
        """
        m = np.minimum(margin, (self.upper - self.lower) / 4)
        lo, hi = self.lower + m, self.upper - m
        q = np.clip(np.array(q0, dtype=float), lo, hi)
        free = np.ones(7, dtype=bool)
        for i, v in (locked or {}).items():
            q[i], free[i] = v, False
        prev_er = math.inf
        for _ in range(iters):
            T, axes = self._frames(q, extra)
            ep_v = T_des[:3, 3] - T[:3, 3]
            er_v = rot_error(T[:3, :3], T_des[:3, :3])
            ep, er = np.linalg.norm(ep_v), np.linalg.norm(er_v)
            if ep < tol_pos and (orient == "none" or er < tol_rot):
                break
            if orient == "secondary" and ep < tol_pos and prev_er - er < 1e-6:
                break               # позиция есть, ориентация больше не улучшается
            prev_er = er
            J = self._jac(T, axes)
            active = free.copy()
            for _ in range(3):      # суставы, упёршиеся в предел, исключаются и шаг пересчитывается
                dq = self._step(J * active, ep_v, er_v, orient, rot_weight, damping) * active
                blocked = active & (((q <= lo + 1e-9) & (dq < 0)) | ((q >= hi - 1e-9) & (dq > 0)))
                if not blocked.any():
                    break
                active &= ~blocked
            n = np.max(np.abs(dq))
            if n > max_step:
                dq *= max_step / n
            q = np.clip(q + dq, lo, hi)
            for i, v in (locked or {}).items():
                q[i] = v
        T = self.fk(q, extra)
        return q, float(np.linalg.norm(T_des[:3, 3] - T[:3, 3])), float(np.linalg.norm(rot_error(T[:3, :3], T_des[:3, :3])))

    def solve(self, T_des, q_now, extra=None, orient="full", rot_weight=1.0, locked=None, margin=0.05,
              n_random=8, tol_pos=0.005, tol_rot=0.03, seed=0):
        """IK из текущей позы, средней позы и случайных стартов.

        Из решений в допуске берётся ближайшее к q_now (меньше движения), для "secondary" — с наименьшей
        ошибкой ориентации; если в допуск не попало ни одно — с наименьшей ошибкой.
        -> (ok, q, ошибка позиции, ошибка ориентации).
        """
        rng = np.random.default_rng(seed)
        mid = (self.lower + self.upper) / 2.0
        seeds = [np.array(q_now, dtype=float), mid] + [rng.uniform(self.lower, self.upper) for _ in range(n_random)]
        best, best_ok = None, None
        for s in seeds:
            q, ep, er = self.ik(T_des, s, extra, orient, rot_weight, locked, margin,
                                tol_pos=tol_pos / 5, tol_rot=tol_rot / 5)
            ok = ep < tol_pos and (orient != "full" or er < tol_rot)
            if ok:
                dist = np.linalg.norm(q - q_now)
                score = er + 0.05 * dist if orient == "secondary" else dist
                if best_ok is None or score < best_ok[0]:
                    best_ok = (score, q, ep, er)
                if s is seeds[0] and (orient != "secondary" or er < tol_rot):
                    break           # из текущей позы сошлось — ближе не найти
            elif best is None or ep + 0.1 * er < best[1] + 0.1 * best[2]:
                best = (q, ep, er)
        if best_ok:
            return True, best_ok[1], best_ok[2], best_ok[3]
        return False, best[0], best[1], best[2]
