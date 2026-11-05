# -*- coding: utf-8 -*-
"""Чистый Python совместимый слой numpy для arm_ik (без pip-зависимостей).

API покрывает ровно то, что используют kinematics/collision/trajectory/nodes:
zeros/eye/asarray/dot/matmul/@/norm/linalg.solve/clip/vstack/tile/linspace/
maximum/min/abs/sign/ceil/array и т.д. Математика double, результаты
эквивалентны numpy (тест test_pure_python проверяет совпадение с настоящим
numpy/scipy, когда они установлены).

Включается автоматически в _math_compat, если numpy/scipy не импортируются,
или принудительно: ARM_IK_FORCE_PURE=1.
"""
import math


class _Array(list):
    """2D/1D массив; numpy-подобная индексация T[:3, 3] и T[:3, :3]."""

    __slots__ = ()

    def _wrapv(self, v):
        if isinstance(v, list) and not isinstance(v, _Array):
            return _Array([x for x in v])
        return v

    @staticmethod
    def _col(row, kk):
        row = row if isinstance(row, list) else [row]
        if isinstance(kk, slice):
            return _Array(row[kk])
        return row[kk]

    # ---------- numpy-подобная индексация [row, col] / срезы ----------
    @staticmethod
    def _sub(row, kk):
        """Применить ключ kk к строке (list/_Array)."""
        if isinstance(kk, slice):
            return _Array([x for x in row[kk]])
        v = row[kk]
        return _Array([x for x in v]) if isinstance(v, list) else v

    def __getitem__(self, key):
        if isinstance(key, tuple):
            cur = list.__getitem__(self, key[0])
            ndim_left = len(key) - 1
            for kk in key[1:]:
                ndim_left -= 1
                if isinstance(cur, (list, _Array)):
                    cur = self._sub(cur, kk)
                else:  # скаляр дальше не индексируется
                    return cur
                # numpy-семантика: A[:, j] -> [row[j], ...]; если после этого
                # размерность ещё есть — оборачиваем каждую строку в _Array
                if ndim_left and isinstance(cur, list) \
                        and cur and isinstance(cur[0], list):
                    cur = _Array([_Array(r) if isinstance(r, list) else r
                                  for r in cur])
            return cur
        v = list.__getitem__(self, key)
        return self._wrapv(v)

    def __setitem__(self, key, val):
        if isinstance(key, tuple):
            rowkey = key[0]
            rows = list.__getitem__(self, rowkey)
            single = not isinstance(rowkey, slice)
            if single:
                rows = [rows]
            src2d = _is2d(val)
            for i, r in enumerate(rows):
                tgt = r
                for kk in key[1:-1]:
                    tgt = tgt[kk]
                colkey = key[-1]
                if isinstance(colkey, slice):
                    src = val[i] if (src2d and len(val) == len(rows)) \
                        else (val if src2d and len(val) == 1 else val)
                    if src2d and len(val) == 1:
                        src = val[0]
                    cols = list(range(len(tgt)))[colkey]
                    for j, c in enumerate(cols):
                        tgt[c] = float(src[j])
                else:
                    if src2d:
                        v = val[i] if len(val) == len(rows) else val[0][colkey]
                    elif isinstance(val, (list, _Array, _P)):
                        v = val[i] if len(rows) > 1 else val[colkey]
                    else:
                        v = val
                    tgt[colkey] = float(v)
            return
        if isinstance(val, (list, tuple, _Array, _P)) and not isinstance(key, slice):
            list.__setitem__(self, key, [float(x) for x in val])
        elif isinstance(key, slice) and isinstance(val, (list, tuple, _Array, _P)):
            idxs = list(range(len(self)))[key]
            for i, t in enumerate(idxs):
                src = val[i]
                cur = list.__getitem__(self, t)
                if isinstance(cur, list):
                    for j in range(len(cur)):
                        cur[j] = float(src[j])
                else:
                    list.__setitem__(self, t, float(src))
        else:
            list.__setitem__(self, key, val)

    # ---------- конструктор/свойства ----------
    @property
    def shape(self):
        s = [len(self)]
        if self and isinstance(self[0], (list, tuple, _Array)):
            s += _Array([*self[0]]).shape if not isinstance(self[0], _Array) else self[0].shape
        return tuple(s)

    @property
    def ndim(self):
        return len(self.shape)

    @property
    def T(self):
        return transpose(self)

    def copy(self):
        return _Array([r.copy() if isinstance(r, (list, _Array)) else r for r in list.__iter__(self)])

    def tolist(self):
        out = []
        for x in self:
            out.append(x.tolist() if isinstance(x, list) else float(x))
        return out

    def flatten(self):
        return _Array([float(x) for row in self for x in (row if isinstance(row, list) else [row])])

    ravel = flatten

    # ---------- арифметика поэлементно ----------
    def _binop(self, other, f):
        def apply(a, b):
            a_seq = isinstance(a, (list, tuple, _Array))
            b_seq = isinstance(b, (list, tuple, _Array))
            if a_seq:
                if b_seq:
                    if len(a) != len(b):
                        raise ValueError("array operands must have matching shapes")
                    return _Array([apply(x, y) for x, y in zip(a, b)])
                return _Array([apply(x, b) for x in a])
            if b_seq:
                raise ValueError("array operands must have matching shapes")
            return f(float(a), float(b))
        return apply(self, other)

    def __add__(self, o):
        return self._binop(o, lambda a, b: a + b)

    def __radd__(self, o):
        return self.__add__(o)

    def __sub__(self, o):
        return self._binop(o, lambda a, b: a - b)

    def __rsub__(self, o):
        return self._binop(o, lambda a, b: b - a)

    def __mul__(self, o):
        return self._binop(o, lambda a, b: a * b)

    def __rmul__(self, o):
        return self.__mul__(o)

    def __truediv__(self, o):
        return self._binop(o, lambda a, b: a / b)

    def __neg__(self):
        return _Array([-x if not isinstance(x, (list, tuple, _Array))
                       else -_wrap(x) for x in self])

    def __abs__(self):
        return _Array([abs(x) if not isinstance(x, (list, tuple, _Array))
                       else abs(_wrap(x)) for x in self])

    # сравнения -> списки bool (для where/any/all)
    def _cmp(self, o, f):
        def compare(a, b):
            a_seq = isinstance(a, (list, tuple, _Array))
            b_seq = isinstance(b, (list, tuple, _Array))
            if a_seq:
                if b_seq:
                    if len(a) != len(b):
                        raise ValueError("comparison operands must have matching shapes")
                    return [compare(x, y) for x, y in zip(a, b)]
                return [compare(x, b) for x in a]
            if b_seq:
                raise ValueError("comparison operands must have matching shapes")
            return f(float(a), float(b))
        return compare(self, o)

    def __le__(self, o):
        return self._cmp(o, lambda a, b: a <= b)

    def __lt__(self, o):
        return self._cmp(o, lambda a, b: a < b)

    def __ge__(self, o):
        return self._cmp(o, lambda a, b: a >= b)

    def __gt__(self, o):
        return self._cmp(o, lambda a, b: a > b)

    def __eq__(self, o):
        return self._cmp(o, lambda a, b: a == b)

    def max(self):
        return max(float(x) for x in self)

    def min(self):
        return min(float(x) for x in self)

    def sum(self):
        return sum(float(x) for x in self)


def _is2d(m):
    return isinstance(m, (list, tuple, _Array, _P)) and len(m) > 0 \
        and isinstance(m[0], (list, tuple, _Array, _P))


def _rows(m):
    return [[float(x) for x in row] for row in m]


def _wrap(x):
    if isinstance(x, (_Array, _P)):
        return x
    if isinstance(x, list):
        return _Array(x)
    if isinstance(x, tuple):
        return _Array(list(x))
    return x


class _NP:
    """Подмножество numpy."""

    pi = math.pi
    inf = math.inf

    # ---------- создание ----------
    @staticmethod
    def array(obj, dtype=None):
        if _is2d(obj):
            return _Array([_Array([float(v) for v in row]) for row in obj])
        if isinstance(obj, (list, tuple, _Array, _P)):
            try:
                return _Array([float(v) for v in obj])
            except TypeError:
                return _Array(_NP.array(list(obj)))  # вложенные списки без единой ширины
        if isinstance(obj, _Array):
            return _Array([float(v) for v in obj._d])
        return _Array([float(obj)])

    asarray = array
    float64 = float

    # ndarray — тот же список-массив (для аннотаций isinstance не используется)
    ndarray = _Array

    @staticmethod
    def append(arr, values):
        return _Array([float(x) for x in arr] + [float(values)]
                      if isinstance(values, (int, float)) else
                      [float(x) for x in arr] + [float(x) for x in values])

    @staticmethod
    def concatenate(seq, axis=0):
        out = []
        for s in seq:
            out += [float(x) for x in s]
        return _Array(out)

    @staticmethod
    def diag(vals):
        n = len(vals)
        return _Array([_Array([float(vals[i]) if i == j else 0.0 for j in range(n)])
                       for i in range(n)])

    @staticmethod
    def allclose(a, b, atol=1e-8, rtol=1e-5):
        def flatten(value):
            out = []
            for item in value:
                if isinstance(item, (list, tuple, _Array)):
                    out.extend(flatten(item))
                else:
                    out.append(float(item))
            return out
        aa, bb = flatten(a), flatten(b)
        if len(aa) != len(bb):
            return False
        return all(abs(x - y) <= atol + rtol * abs(y)
                   for x, y in zip(aa, bb))

    @staticmethod
    def zeros(n, dtype=None):
        if isinstance(n, (tuple, list)):
            r, c = n
            return _Array([_Array([0.0] * c) for _ in range(r)])
        return _Array([0.0] * int(n))

    @staticmethod
    def ones(n, dtype=None):
        return _Array([1.0] * int(n))

    @staticmethod
    def full(shape, val, dtype=None):
        if isinstance(shape, (tuple, list)):
            r, c = shape
            return _Array([_Array([float(val)] * c) for _ in range(r)])
        return _Array([float(val)] * int(shape))

    @staticmethod
    def eye(n, dtype=None):
        return _Array([_Array([1.0 if i == j else 0.0 for j in range(n)]) for i in range(n)])

    @staticmethod
    def linspace(a, b, N):
        if N <= 1:
            return _Array([float(a)])
        step = (b - a) / (N - 1)
        return _Array([a + step * i for i in range(N)])

    # ---------- элементарные функции ----------
    @staticmethod
    def sqrt(x):
        if isinstance(x, (list, tuple, _Array)):
            return _Array([math.sqrt(max(0.0, float(v))) for v in x])
        return math.sqrt(max(0.0, float(x)))

    @staticmethod
    def exp(x):
        if isinstance(x, (list, tuple, _Array)):
            return _Array([math.exp(float(v)) for v in x])
        return math.exp(float(x))

    @staticmethod
    def maximum(a, b):
        if isinstance(a, (list, tuple, _Array)):
            if isinstance(b, (list, tuple, _Array)):
                return _Array([max(float(x), float(y)) for x, y in zip(a, b)])
            return _Array([max(float(x), float(b)) for x in a])
        return max(float(a), float(b))

    @staticmethod
    def minimum(a, b):
        if isinstance(a, (list, tuple, _Array)):
            if isinstance(b, (list, tuple, _Array)):
                return _Array([min(float(x), float(y)) for x, y in zip(a, b)])
            return _Array([min(float(x), float(b)) for x in a])
        return min(float(a), float(b))

    @staticmethod
    def clip(v, lo, hi):
        values = list(v)
        lows = list(lo) if isinstance(lo, (list, tuple, _Array)) else [lo] * len(values)
        highs = list(hi) if isinstance(hi, (list, tuple, _Array)) else [hi] * len(values)
        if len(lows) != len(values) or len(highs) != len(values):
            raise ValueError("clip bounds must be scalar or match the input length")
        return _Array([min(float(h), max(float(l), float(x)))
                       for x, l, h in zip(values, lows, highs)])

    @staticmethod
    def sign(x):
        if isinstance(x, (list, tuple, _Array)):
            return _Array([math.copysign(1.0, float(v)) if float(v) != 0 else 0.0 for v in x])
        return math.copysign(1.0, x) if x != 0 else 0.0

    @staticmethod
    def abs(x):
        if isinstance(x, (list, tuple, _Array)):
            return _Array([abs(float(v)) for v in x])
        return abs(float(x))

    @staticmethod
    def ceil(x):
        return math.ceil(float(x))

    @staticmethod
    def floor(x):
        return math.floor(float(x))

    @staticmethod
    def norm(v, ord=None):
        return math.sqrt(sum(float(x) * float(x) for x in v))

    @staticmethod
    def cross(u, v):
        return _Array([u[1] * v[2] - u[2] * v[1],
                       u[2] * v[0] - u[0] * v[2],
                       u[0] * v[1] - u[1] * v[0]])

    @staticmethod
    def dot(a, b):
        a, b = _wrap(a), _wrap(b)
        if _is2d(a) and _is2d(b):
            ra, ca, cb = len(a), len(a[0]), len(b[0])
            out = [[0.0] * cb for _ in range(ra)]
            for i in range(ra):
                ai = a[i]
                for k in range(ca):
                    aik = float(ai[k])
                    if aik == 0.0:
                        continue
                    bk = b[k]
                    oi = out[i]
                    for j in range(cb):
                        oi[j] += aik * float(bk[j])
            return _P(_Array([_Array(r) for r in out]))
        if _is2d(a):  # M @ v
            return _P([sum(float(ai[k]) * float(b[k]) for k in range(len(b))) for ai in a])
        if _is2d(b):  # v @ M
            return _P([sum(float(a[i]) * float(b[i][j]) for i in range(len(a)))
                           for j in range(len(b[0]))])
        return sum(float(x) * float(y) for x, y in zip(a, b))

    @staticmethod
    def where(cond, a, b):
        out = []
        for c, x, y in zip(cond, a, b):
            out.append(float(x) if c else float(y))
        return _Array(out)

    @staticmethod
    def any(seq):
        return any(bool(x) for x in seq)

    @staticmethod
    def all(seq):
        return all(bool(x) for x in seq)

    @staticmethod
    def stack(seq, axis=0):
        assert axis == 0
        return _Array([_wrap(_Array(list(r))).copy() for r in seq])

    @staticmethod
    def vstack(seq):
        out = []
        for m in seq:
            m = _wrap(m)
            if _is2d(m):
                out += [_Array(list(r)) for r in m]
            else:
                out.append(_Array([float(x) for x in m]))
        return _Array(out)

    @staticmethod
    def hstack(seq):
        rows = max(len(m) for m in seq)
        out = []
        for i in range(rows):
            row = []
            for m in seq:
                if _is2d(m):
                    row += [float(x) for x in m[i]]
                else:
                    row.append(float(m[i]))
            out.append(_Array(row))
        return _Array(out)

    @staticmethod
    def tile(vec, reps):
        n, times = reps
        return _Array([_Array([float(x) for x in vec]) for _ in range(times)])

    @staticmethod
    def diff(m, axis=0):
        assert axis == 0 and _is2d(m)
        return _Array([_Array([float(m[i + 1][j]) - float(m[i][j]) for j in range(len(m[0]))])
                       for i in range(len(m) - 1)])

    @staticmethod
    def sum(x, axis=None):
        if _is2d(x) and axis is None:
            return sum(float(v) for row in x for v in row)
        if _is2d(x) and axis == 0:
            return _Array([sum(float(row[j]) for row in x) for j in range(len(x[0]))])
        return sum(float(v) for v in x)

    @staticmethod
    def max(x, axis=None):
        if _is2d(x):
            return max(float(v) for row in x for v in row)
        return max(float(v) for v in x)

    @staticmethod
    def min(x, axis=None):
        if _is2d(x):
            return min(float(v) for row in x for v in row)
        return min(float(v) for v in x)

    class linalg:
        @staticmethod
        def inv(A):
            n = len(A)
            Aug = [_Array([float(A[i][j]) for j in range(n)] +
                          [1.0 if i == j else 0.0 for j in range(n)]) for i in range(n)]
            for col in range(n):
                piv = max(range(col, n), key=lambda r: abs(Aug[r][col]))
                if abs(Aug[piv][col]) < 1e-14:
                    raise ZeroDivisionError("singular matrix")
                Aug[col], Aug[piv] = Aug[piv], Aug[col]
                pv = float(Aug[col][col])
                Aug[col] = _Array([v / pv for v in Aug[col]])
                for r in range(n):
                    if r != col and float(Aug[r][col]) != 0.0:
                        f = float(Aug[r][col])
                        Aug[r] = _Array([float(Aug[r][k]) - f * float(Aug[col][k])
                                         for k in range(2 * n)])
            return _Array([_Array([float(Aug[i][n + j]) for j in range(n)]) for i in range(n)])

        @staticmethod
        def solve(A, b):
            # Гаусс с выбором ведущего элемента; A — 2D, b — 1D или 2D
            n = len(A)
            Ab = [_Array([float(v) for v in A[i]] +
                         ([float(b[i])] if not _is2d(b) else [float(x) for x in b[i]]))
                  for i in range(n)]
            nb = 1 if not _is2d(b) else len(b[0])
            for col in range(n):
                piv = max(range(col, n), key=lambda r: abs(Ab[r][col]))
                if abs(Ab[piv][col]) < 1e-14:
                    raise ZeroDivisionError("singular matrix")
                Ab[col], Ab[piv] = Ab[piv], Ab[col]
                pv = float(Ab[col][col])
                Ab[col] = _Array([v / pv for v in Ab[col]])
                for r in range(n):
                    if r != col and float(Ab[r][col]) != 0.0:
                        f = float(Ab[r][col])
                        Ab[r] = _Array([float(Ab[r][k]) - f * float(Ab[col][k])
                                        for k in range(n + nb)])
            if nb == 1:
                return _Array([float(Ab[i][n]) for i in range(n)])
            return _Array([_Array([float(Ab[i][n + j]) for j in range(nb)]) for i in range(n)])

        @staticmethod
        def pinv(A, rcond=1e-10):
            # псевдообратная через нормальные уравнения с регуляризацией НЕ нужна:
            # считаем J^+ = A^T (A A^T)^{-1} для тонких A (6x7) — как в коде пакета.
            # Универсально: SVD не реализуем; используем устойчивый путь для A(m,n), m<n.
            m, n = len(A), len(A[0])
            AAt = _NP.dot(A, transpose(A))
            reg = 1e-8 * _NP.eye(m)
            inv = _NP.linalg.solve(_NP.array([[AAt[i][j] + reg[i][j] for j in range(m)]
                                              for i in range(m)]),
                                   _NP.eye(m))
            return _NP.dot(transpose(A), inv)

    class random:
        @staticmethod
        def default_rng(seed=None):
            import random as _r

            rng = _r.Random(seed)

            class _R:
                def uniform(self_, lo, hi, size=None):
                    if size is None:
                        return rng.uniform(lo, hi)
                    return _Array([rng.uniform(lo, hi) for _ in range(size)])

                def normal(self_, mu, sd, size=None):
                    if size is None:
                        return rng.gauss(mu, sd)
                    return _Array([rng.gauss(mu, sd) for _ in range(size)])

            return _R()


class _P(_Array):
    """Результат np.dot и т.п.: тот же класс, что и массивы (numpy-индексация)."""

    __slots__ = ()


def transpose(M):
    return _Array([_Array([float(M[i][j]) for i in range(len(M))]) for j in range(len(M[0]))])


# оператор @ для _Array (matmul)
def _matmul(a, b):
    r = _NP.dot(a, b)
    return r

_Array.__matmul__ = _matmul
_Array.__rmatmul__ = lambda self, o: _NP.dot(o, self)

np = _NP()
