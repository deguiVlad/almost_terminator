# -*- coding: utf-8 -*-
"""Коллизийный страж: стол как горизонтальная полуплоскость + самопроверка позы.

Стол в сцене — рабочая поверхность перед роботом на высоте h_table над pelvis
(параметр узла; измерить один раз по TF/линейке или взять из мёртвых поз объектов).
Запрещаем любой точке кинематической цепочки руки опускаться ниже z = h_table
внутри прямоугольника стола (x ∈ [x0, x1], y ∈ [-y1, y1]).

Два инструмента:
  * table_cost_gradient(q) — мягкий штраф (барьер), добавляемый в IK-шаг:
        dq -= k_barrier * dCost/dq   (только если точка ниже барьера)
  * collides_with_table(q) — жёсткая проверка для валидации траектории.

Для строгой проверки «рука не задела стол» можно подключить FCL через
MoveIt / `moveit_msgs/srv/CheckCollisionFine`, но для плоского стола
аналитического теста достаточно и он работает на 50 Гц без нагрузки на CPU.
"""
from arm_ik._math_compat import np

from .kinematics import chain_transforms


class TableGuard:
    def __init__(self, h_table=0.9, x0=0.35, x1=1.2, y_half=1.0, margin=0.02):
        values = (h_table, x0, x1, y_half, margin)
        if not all(np.isfinite(float(v)) for v in values):
            raise ValueError("table geometry parameters must be finite")
        if x0 >= x1 or y_half <= 0.0 or margin < 0.0:
            raise ValueError("invalid table bounds or margin")
        self.h = float(h_table)
        self.x0, self.x1 = float(x0), float(x1)
        self.y_half = float(y_half)
        self.margin = float(margin)

    def inside_xy(self, p):
        return (self.x0 <= p[0] <= self.x1) and (abs(p[1]) <= self.y_half)

    def penetration(self, p, link_radius=0.0):
        """Penetration depth with an optional conservative XY link radius."""
        x, y, z = (float(v) for v in p)
        inside = (self.x0 - link_radius <= x <= self.x1 + link_radius
                  and abs(y) <= self.y_half + link_radius)
        if inside:
            return max(0.0, (self.h + self.margin) - z)
        return 0.0

    def _points(self, model, q, T_guard_model=None):
        """Joint/TCP points in the table-guard frame."""
        Ts = chain_transforms(model, q)
        points = [np.zeros(3)] + [T[:3, 3] for T in Ts]
        points.append((Ts[-1] @ np.append(model.tcp_offset, 1.0))[:3])
        if T_guard_model is None:
            return points
        transform = np.asarray(T_guard_model, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("T_guard_model must be a finite 4x4 transform")
        return [(transform @ np.append(point, 1.0))[:3] for point in points]

    def collides_with_table(self, model, q, link_radius=0.05, sample_step=0.025,
                            T_guard_model=None):
        """Sample link segments in the table frame with a conservative radius.

        This is a planar-table approximation, not full MoveIt/FCL collision
        geometry. T_guard_model maps the URDF model base into the table frame.
        """
        if (not np.isfinite(float(link_radius)) or not np.isfinite(float(sample_step))
                or link_radius < 0.0 or sample_step <= 0.0):
            raise ValueError("link_radius must be finite and >= 0; sample_step finite and > 0")
        pts = self._points(model, q, T_guard_model)

        def collides(p):
            x, y, z = (float(v) for v in p)
            inside = (self.x0 - link_radius <= x <= self.x1 + link_radius
                      and abs(y) <= self.y_half + link_radius)
            return inside and z - link_radius < self.h + self.margin

        for a, b in zip(pts[:-1], pts[1:]):
            delta = b - a
            length = float(np.linalg.norm(delta))
            count = max(1, int(np.ceil(length / sample_step)))
            for i in range(count + 1):
                p = a + delta * (float(i) / count)
                if collides(p):
                    return True
        return False

    def table_cost_gradient(self, model, q, k=50.0, link_radius=0.05,
                            T_guard_model=None):
        """Numerical gradient of the table penalty in the table-guard frame."""
        if not np.isfinite(float(k)) or k < 0.0:
            raise ValueError("barrier gain must be finite and non-negative")
        pts = self._points(model, q, T_guard_model)
        n = len(model.joints)
        dq = np.zeros(n)
        eps = 1e-4
        base_cost = self._cost(pts, link_radius)
        if base_cost <= 0.0:
            return dq
        for i in range(n):
            qp = np.asarray(q, dtype=float).copy()
            qp[i] += eps
            ptsp = self._points(model, qp, T_guard_model)
            dq[i] = (self._cost(ptsp, link_radius) - base_cost) / eps
        return -k * dq

    def _cost(self, pts, link_radius):
        c = 0.0
        for p in pts:
            d = self.penetration(p - np.array([0, 0, link_radius]), link_radius)
            c += d * d
        return c

    def blocks_T(self, T44):
        """True, если позиция из 4x4 позы попадает под стол (с запасом)."""
        p = [float(v) for v in T44[:3, 3]]
        return self.penetration(np.array(p), link_radius=0.05) > 0.0

    def clamp_goal(self, T44):
        """Возвращает копию T44 с z цели, поднятым над столом+запас."""
        T = np.array(T44, dtype=float).copy()
        p = self.clamp_target_above_table([float(v) for v in T[:3, 3]])
        T[:3, 3] = p
        return T

    def clamp_target_above_table(self, pos, link_radius=0.05):
        """Поднять TCP так, чтобы условный радиус инструмента был выше стола."""
        p = np.asarray(pos, dtype=float).copy()
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError("target position must contain three finite values")
        if not np.isfinite(float(link_radius)) or link_radius < 0.0:
            raise ValueError("link_radius must be finite and non-negative")
        expanded_inside = (
            self.x0 - link_radius <= p[0] <= self.x1 + link_radius
            and abs(p[1]) <= self.y_half + link_radius)
        if expanded_inside:
            p[2] = max(p[2], self.h + self.margin + float(link_radius))
        return p
