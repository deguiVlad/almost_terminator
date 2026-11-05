# -*- coding: utf-8 -*-
"""Синхронизированный трапецеидальный профиль суставов с лимитами скорости/ускорения."""
import math

from arm_ik._math_compat import np


def joint_space_profile(q0, q1, v_max=0.8, a_max=0.4, dt=0.02):
    """Вернуть (N, n) траекторию с ограничениями |dq/dt| <= v_max и |ddq/dt²| <= a_max.

    Все суставы завершают движение одновременно. Временная сетка имеет ровный
    шаг dt, а длительность автоматически округляется вверх до целого числа
    шагов — конечные разности проверяются на том же dt, который использует узел.
    """
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if len(q0) == 0 or len(q0) != len(q1):
        raise ValueError("q0 and q1 must have the same non-zero length")
    if not all(math.isfinite(float(x)) for x in list(q0) + list(q1)):
        raise ValueError("joint positions must be finite")
    if not (math.isfinite(v_max) and v_max > 0.0):
        raise ValueError("v_max must be finite and > 0")
    if not (math.isfinite(a_max) and a_max > 0.0):
        raise ValueError("a_max must be finite and > 0")
    if not (math.isfinite(dt) and dt > 0.0):
        raise ValueError("dt must be finite and > 0")

    delta = q1 - q0
    distances = [abs(float(x)) for x in delta]
    if max(distances, default=0.0) <= 1e-12:
        return np.vstack([q0, q1])

    axis_times = []
    for distance in distances:
        if distance <= 1e-12:
            axis_times.append(0.0)
        elif distance <= (v_max * v_max / a_max):
            axis_times.append(2.0 * math.sqrt(distance / a_max))
        else:
            axis_times.append(distance / v_max + v_max / a_max)

    min_duration = max(axis_times)
    steps = max(1, int(math.ceil(min_duration / dt)))
    duration = steps * dt
    times = [k * dt for k in range(steps + 1)]
    trajectory = []

    for t in times:
        q = []
        for i, distance in enumerate(distances):
            if distance <= 1e-12:
                q.append(float(q1[i]))
                continue

            accel_time = min(v_max / a_max, duration / 2.0)
            peak_velocity = distance / (duration - accel_time)
            accel = peak_velocity / accel_time
            if peak_velocity > v_max + 1e-9 or accel > a_max + 1e-9:
                raise RuntimeError("internal trajectory timing violates motion limits")

            if t <= accel_time:
                travelled = 0.5 * accel * t * t
            elif t <= duration - accel_time:
                travelled = 0.5 * peak_velocity * accel_time + peak_velocity * (t - accel_time)
            else:
                remaining = duration - t
                travelled = distance - 0.5 * accel * remaining * remaining

            sign = 1.0 if float(delta[i]) >= 0.0 else -1.0
            q.append(float(q0[i]) + sign * travelled)
        trajectory.append(q)

    trajectory[-1] = [float(x) for x in q1]
    return np.asarray(trajectory, dtype=float)
