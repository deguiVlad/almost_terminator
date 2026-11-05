# -*- coding: utf-8 -*-
"""Чистый Python fallback для numpy/scipy (матрицы 4x4, кватернионы, эйлеры).

Предназначен ТОЛЬКО для запуска ROS-нод в окружениях, где pip-установка
numpy/scipy недоступна (чужие venv/оверлеи). Точность — double, всё линейное
(7 DOF), поэтому чистый Python здесь полностью эквивалентен numpy по результату.

Публичный API имитирует нужное подмножество:
    np.zeros(n) / np.eye(4) / np.dot(A,B) / np.asarray(x).tolist()
    Rotation.from_euler/as_rotvec/from_matrix/from_quat + .as_*()
Если настоящий numpy+scipy доступны — используется ОНИ (быстрее), этот модуль
тогда просто проксирует. Модули пакета делают:

    from arm_ik._math_compat import np, Rotation

и не думают об окружении.
"""
import math
import os

HAVE_REAL = False
try:  # приоритет — настоящие библиотеки
    import numpy as _np_real
    from scipy.spatial.transform import Rotation as _Rot_real

    if os.environ.get("ARM_IK_FORCE_PURE", "") == "1":
        raise ImportError("pure-python mode requested")
    np = _np_real
    Rotation = _Rot_real
    HAVE_REAL = True
except Exception:  # окружение без numpy/scipy — чистый Python fallback
    HAVE_REAL = False

    from arm_ik import _pure_np as _pn

    np = _pn._NP

    # ---------------- Rotation: подмножество scipy.spatial.transform ----------------
    def _mat_from_euler_xyz(rpy):
        cx, sx = math.cos(rpy[0]), math.sin(rpy[0])
        cy, sy = math.cos(rpy[1]), math.sin(rpy[1])
        cz, sz = math.cos(rpy[2]), math.sin(rpy[2])
        # intrinsic xyz == R = Rz @ Ry @ Rx (совпадает со scipy)
        return [[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                [-sy, cy * sx, cy * cx]]

    def _rotvec_to_matrix(rv):
        ang = math.sqrt(sum(c * c for c in rv))
        if ang < 1e-12:
            return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        kx, ky, kz = (c / ang for c in rv)
        c, s = math.cos(ang), math.sin(ang)
        t = 1.0 - c
        return [[t * kx * kx + c, t * kx * ky - s * kz, t * kx * kz + s * ky],
                [t * kx * ky + s * kz, t * ky * ky + c, t * ky * kz - s * kx],
                [t * kx * kz - s * ky, t * ky * kz + s * kx, t * kz * kz + c]]

    def _matrix_to_rotvec(Rm):
        tr = Rm[0][0] + Rm[1][1] + Rm[2][2]
        cosang = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
        ang = math.acos(cosang)
        if ang < 1e-9:
            return [0.0, 0.0, 0.0]
        if math.pi - ang < 1e-6:
            xx = (Rm[0][0] + 1.0) / 2.0
            yy = (Rm[1][1] + 1.0) / 2.0
            zz = (Rm[2][2] + 1.0) / 2.0
            x = math.sqrt(max(xx, 0.0))
            y = math.sqrt(max(yy, 0.0))
            z = math.sqrt(max(zz, 0.0))
            if Rm[0][1] + Rm[1][0] < 0:
                y = -y
            if Rm[0][2] + Rm[2][0] < 0:
                z = -z
            n = ang / math.sqrt(max(x * x + y * y + z * z, 1e-12))
            return [x * n, y * n, z * n]
        s = ang / (2.0 * math.sin(ang))
        return [s * (Rm[2][1] - Rm[1][2]),
                s * (Rm[0][2] - Rm[2][0]),
                s * (Rm[1][0] - Rm[0][1])]

    def _matrix_to_quat_xyzw(Rm):
        tr = Rm[0][0] + Rm[1][1] + Rm[2][2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (Rm[2][1] - Rm[1][2]) / S
            y = (Rm[0][2] - Rm[2][0]) / S
            z = (Rm[1][0] - Rm[0][1]) / S
        elif Rm[0][0] > Rm[1][1] and Rm[0][0] > Rm[2][2]:
            S = math.sqrt(1.0 + Rm[0][0] - Rm[1][1] - Rm[2][2]) * 2.0
            w = (Rm[2][1] - Rm[1][2]) / S
            x = 0.25 * S
            y = (Rm[0][1] + Rm[1][0]) / S
            z = (Rm[0][2] + Rm[2][0]) / S
        elif Rm[1][1] > Rm[2][2]:
            S = math.sqrt(1.0 + Rm[1][1] - Rm[0][0] - Rm[2][2]) * 2.0
            w = (Rm[0][2] - Rm[2][0]) / S
            x = (Rm[0][1] + Rm[1][0]) / S
            y = 0.25 * S
            z = (Rm[1][2] + Rm[2][1]) / S
        else:
            S = math.sqrt(1.0 + Rm[2][2] - Rm[0][0] - Rm[1][1]) * 2.0
            w = (Rm[1][0] - Rm[0][1]) / S
            x = (Rm[0][2] + Rm[2][0]) / S
            y = (Rm[1][2] + Rm[2][1]) / S
            z = 0.25 * S
        n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
        return _pn._Array([x / n, y / n, z / n, w / n])

    def _quat_xyzw_to_matrix(q):
        x, y, z, w = q
        return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]

    class _Rotation:
        """Подмножество scipy Rotation: from_euler('xyz'), from_rotvec,
        from_matrix, from_quat; as_matrix(), as_rotvec(), as_quat()."""

        def __init__(self, mat):
            self._m = [row[:] for row in mat]

        @classmethod
        def from_euler(cls, seq, angles, degrees=False):
            assert seq == "xyz", "fallback поддерживает только последовательность 'xyz'"
            a = list(map(float, angles))
            if degrees:
                a = [math.radians(t) for t in a]
            return cls(_mat_from_euler_xyz(a))

        @classmethod
        def from_rotvec(cls, rv):
            return cls(_rotvec_to_matrix(list(map(float, rv))))

        @classmethod
        def from_matrix(cls, m):
            return cls([[float(x) for x in row[:3]] for row in m[:3]])

        @classmethod
        def from_quat(cls, q):
            return cls(_quat_xyzw_to_matrix(list(map(float, q))))

        def as_matrix(self):
            return _pn._Array([_pn._Array(row) for row in self._m])

        def as_rotvec(self):
            return _pn._Array(_matrix_to_rotvec(self._m))

        def as_euler(self, seq, degrees=False):
            """Return intrinsic XYZ Euler angles, matching scipy Rotation."""
            if seq != "xyz":
                raise NotImplementedError("fallback supports only Euler sequence 'xyz'")
            m = self._m
            cy = math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])
            if cy > 1e-9:
                x = math.atan2(m[2][1], m[2][2])
                y = math.atan2(-m[2][0], cy)
                z = math.atan2(m[1][0], m[0][0])
            else:
                x = math.atan2(-m[1][2], m[1][1])
                y = math.atan2(-m[2][0], cy)
                z = 0.0
            angles = [x, y, z]
            if degrees:
                angles = [math.degrees(v) for v in angles]
            return _pn._Array(angles)

        def as_quat(self):
            return _matrix_to_quat_xyzw(self._m)

    Rotation = _Rotation


def T_pos_rot(pos, R33):
    """Собрать 4x4 из позиции и матрицы вращения (общий helper)."""
    T = np.eye(4)
    for i in range(3):
        T[i][3] = float(pos[i])
        for j in range(3):
            T[i][j] = float(R33[i][j])
    return T
