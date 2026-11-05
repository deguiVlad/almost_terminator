# -*- coding: utf-8 -*-
"""Коллизийный страж: стол как горизонтальная полуплоскость + самопроверка позы.

Стол в сцене — рабочая поверхность перед роботом на высоте h_table над base_frame
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
import math

from arm_ik._math_compat import np

from .kinematics import chain_transforms


class TableGuard:
    def __init__(self, h_table=0.9, x0=0.35, x1=1.2, y_half=1.0, margin=0.02):
        values = (h_table, x0, x1, y_half, margin)
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("table geometry parameters must be finite")
        if x0 >= x1 or y_half <= 0.0 or margin < 0.0:
            raise ValueError("require x0 < x1, y_half > 0 and margin >= 0")
        self.h = float(h_table)
        self.x0, self.x1 = float(x0), float(x1)
        self.y_half = float(y_half)
        self.margin = float(margin)

    def inside_xy(self, p):
        return (self.x0 <= p[0] <= self.x1) and (abs(p[1]) <= self.y_half)

    def penetration(self, p):
        """Глубина проникновения точки под стол (0 если всё ок)."""
        if self.inside_xy(p):
            d = (self.h + self.margin) - p[2]
            return max(0.0, d)
        return 0.0

    def collides_with_table(self, model, q, link_radius=0.05):
        """Проверяет все звенья руки (с аппроксимацией «толщиной» link_radius)."""
        Ts = chain_transforms(model, q)
        pts = [np.zeros(3)] + [T[:3, 3] for T in Ts]
        tcp = (Ts[-1] @ np.append(model.tcp_offset, 1.0))[:3]
        pts.append(tcp)
        # Проверяем сегменты, а не только концы: звено может пересечь стол
        # между двумя безопасными по высоте суставными точками.
        samples = []
        for a, b in zip(pts[:-1], pts[1:]):
            length = float(np.linalg.norm(b - a))
            count = max(1, int(math.ceil(length / 0.02)))
            for k in range(count + 1):
                samples.append(a + (b - a) * (float(k) / count))
        for p in samples:
            if self.penetration(p - np.array([0, 0, link_radius])) > 0:
                return True
        return False

    def table_cost_gradient(self, model, q, k=50.0, link_radius=0.05):
        """Мягкий барьер: градиент штрафа по q (7,), выталкивает звенья из стола.

        Cost = Σ_i max(0, depth_i)^2 * k ; dq_step = -J_coll^T * dC/dp_i.
        Возвращает dq_correction (7,) — добавить к шагу IK.
        """
        Ts = chain_transforms(model, q)
        pts = [np.zeros(3)] + [T[:3, 3] for T in Ts]
        tcp = (Ts[-1] @ np.append(model.tcp_offset, 1.0))[:3]
        pts.append(tcp)
        n = len(model.joints)
        dq = np.zeros(n)
        # упрощённый градиент: численно через малые возмущения (стабильно и коротко)
        eps = 1e-4
        base_cost = self._cost(pts, link_radius)
        if base_cost <= 0:
            return dq
        for i in range(n):
            qp = np.asarray(q, dtype=float).copy()
            qp[i] += eps
            Tsp = chain_transforms(model, qp)
            ptsp = [np.zeros(3)] + [T[:3, 3] for T in Tsp]
            ptsp.append((Tsp[-1] @ np.append(model.tcp_offset, 1.0))[:3])
            dq[i] = (self._cost(ptsp, link_radius) - base_cost) / eps
        return -k * dq

    def _cost(self, pts, link_radius):
        # Тот же sampling, что и в жёсткой проверке траектории, иначе мягкий
        # барьер не замечает проникновение середины длинного звена.
        c = 0.0
        samples = []
        for a, b in zip(pts[:-1], pts[1:]):
            length = float(np.linalg.norm(b - a))
            count = max(1, int(math.ceil(length / 0.02)))
            for k in range(count + 1):
                samples.append(a + (b - a) * (float(k) / count))
        for p in samples:
            d = self.penetration(p - np.array([0, 0, link_radius]))
            c += d * d
        return c

    def blocks_T(self, T44):
        """True, если позиция из 4x4 позы попадает под стол (с запасом)."""
        p = [float(v) for v in T44[:3, 3]]
        return self.penetration(np.array(p) - np.array([0.0, 0.0, 0.05])) > 0.0

    def clamp_goal(self, T44):
        """Возвращает копию T44 с z цели, поднятым над столом+запас."""
        import copy as _copy
        T = np.array(T44, dtype=float).copy()
        p = self.clamp_target_above_table([float(v) for v in T[:3, 3]])
        T[:3, 3] = p
        return T

    def clamp_target_above_table(self, pos):
        """Жёсткая страховка цели: TCP никогда не запрашивается ниже стола+запас."""
        p = np.asarray(pos, dtype=float).copy()
        if self.inside_xy(p):
            p[2] = max(p[2], self.h + self.margin + 0.05)
        return p
