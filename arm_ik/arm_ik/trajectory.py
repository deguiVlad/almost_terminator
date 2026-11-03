# -*- coding: utf-8 -*-
"""Планирование позы в позу с ограничением скорости 0.8 рад/с (лимит моторов рук).

Получив q_goal из IK, нельзя просто отправить cmd_pos на полной скорости:
нужен плавный траекторный профиль, совместимый с ПД-приводами и лимитом
0.8 рад/с из документации. Здесь — trapezoidal velocity profile по каждому
суставу + синхронизация по самой долгой оси; генерация промежуточных целей
с частотой publish_rate узла-исполнителя.
"""
import numpy as np


def joint_space_profile(q0, q1, v_max=0.8, a_max=0.4, dt=0.02):
    """Trapezoidal профиль в пространстве суставов, синхронизированный по всем осям.

    Гарантирует |dq/dt| <= v_max на каждой дискретной точке профиля.
    Возвращает массив (N, n_joints), включая q0 и q1.
    """
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    dq = q1 - q0
    d = np.abs(dq)
    # минимальное время на каждую ось: треугольник или трапеция
    t_axis = np.where(d <= v_max ** 2 / a_max,
                      2.0 * np.sqrt(np.maximum(d, 1e-12) / a_max),
                      d / v_max + v_max / a_max)
    T = float(max(t_axis.max(), dt))
    N = int(np.ceil(T / dt)) + 1
    ts = np.linspace(0.0, T, N)
    traj = np.tile(q0, (N, 1))
    for i in range(len(dq)):
        di = d[i]
        if di < 1e-12:
            traj[:, i] = q1[i]
            continue
        ta = min(v_max / a_max, T / 2.0)   # фаза разгона/торможения при общем T
        vc = min(di / (T - ta), v_max)     # крейсерская скорость этой оси
        sign = np.sign(dq[i])
        pos = np.zeros(N)
        for k, t in enumerate(ts):
            if t <= ta:
                s = 0.5 * (vc / ta) * t * t
            elif t <= T - ta:
                s = 0.5 * vc * ta + vc * (t - ta)
            else:
                tau = T - t
                s = di - 0.5 * (vc / ta) * tau * tau
            pos[k] = s
        traj[:, i] = q0[i] + sign * pos
        traj[-1, i] = q1[i]
    return traj
