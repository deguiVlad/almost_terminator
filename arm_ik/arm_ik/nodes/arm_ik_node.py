# -*- coding: utf-8 -*-
"""ROS 2 узел обратной кинематики обеих рук TienKung 2.0 Pro (7 DOF на руку).

Кинематическая модель и limits строятся из URDF; TF2 используется только
для преобразования целевых поз между рамками. Время —
из /clock (запускается с use_sim_time, проверка источника времени выполняется
на старте ноды).

Интерфейсы:
  ВХОД (два способа задать позу кисти):
    * /arm_ik/goal_left, /arm_ik/goal_right   (geometry_msgs/PoseStamped)
        АБСОЛЮТНАЯ шестимерная поза TCP кисти (позиция + ориентация).
        Рамка header.frame_id может быть любой доступной в TF; узел переносит
        цель в base_frame. Пустая/base_frame — без дополнительного преобразования.
    * /arm_ik/delta_left, /arm_ik/delta_right (geometry_msgs/TwistStamped)
        ОТНОСИТЕЛЬНОЕ шестимерное смещение: linear = dxyz (м),
        angular = ось*угол (рад) — rotation vector, не угловая скорость.
        Цель = текущая измеренная поза TCP (FK по /joint_states) + смещение.
        Опции поведения — параметры delta_reference / delta_accumulate /
        delta_max_* (подробности в README пакета).
  ВЫХОД:
    * /arm/cmd_pos            (bodyctrl_msgs/CmdSetMotorPosition) — команды моторов
                              11..17 (левая) и 21..27 (правая), spd <= 0.8 рад/с.
    * /arm_ik/wrist_{left,right}_pose (PoseStamped, рамка base_frame) — текущая поза
                              TCP по FK от /joint_states (видно в RViz, полезно
                              верхнему уровню как «где сейчас кисть»).
    * /arm_ik/{left,right}/status (std_msgs/String, JSON) — фаза контура:
                              idle | traj | fine | reached | failed + ошибки.

Конвейер исполнения цели (подробно — README пакета):
  1. TF-перенос цели в base_frame, клампинг выше стола (TableGuard);
  2. IK: DLS из текущей измеренной позы + сиды-альтернативы (multi-seed),
     мягкий барьер от стола внутри шага IK;
  3. траектория: trapezoidal профиль |dq/dt| <= 0.8 рад/с, валидация каждой
     точки на коллизию со столом; при конфликте — заход через waypoint над
     столом (две фазы профиля);
  4. исполнение на rate_hz: пачка SetMotorPosition в /arm/cmd_pos;
  5. фаза «fine»: замыкание контура по /joint_states — P-коррекция недолёта
     ПД-приводов (до 15 % по docs/INTERFACES.md) шагом dq = J+ e, клип
     ±0.1 рад/цикл (рекомендация FOCUS_TRACKING.md §3), пока ошибка позы
     не войдёт в tol_pos/tol_rot.

Обе руки исполняются независимыми станми в одном узле: топик команд общий,
но ID моторов не пересекаются.
"""
import json
import math
import time

from arm_ik._math_compat import np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from arm_ik._math_compat import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import tf2_ros

try:  # штатный путь: кастомные сообщения робота (bodyctrl) доступны в окружении
    from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
except ImportError:  # graceful degradation: нода стартует и считает IK даже без
    # bodyctrl_msgs (полезно для офлайн-отладки/CI). Команды моторов при этом
    # НЕ публикуются — в лог выдаётся явное предупреждение один раз.
    CmdSetMotorPosition = None
    SetMotorPosition = None

from arm_ik.kinematics import (fk, geometric_jacobian, ik_multi_seed,
                               clip_to_limits, make_default_arm, pose_error,
                               delta_to_T)
from arm_ik.collision import TableGuard
from arm_ik.trajectory import joint_space_profile

def pose_to_T(msg_pose):
    p = msg_pose.position
    q = msg_pose.orientation
    quat = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    pos = np.asarray([p.x, p.y, p.z], dtype=float)
    if not np.all(np.isfinite(quat)) or not np.all(np.isfinite(pos)):
        raise ValueError("pose contains NaN or infinity")
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        raise ValueError("pose quaternion has zero norm")
    quat = quat / norm
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat).as_matrix()
    T[:3, 3] = pos
    return T


def T_to_posemsg(T):
    ps = PoseStamped()
    pos = T[:3, 3]
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos[0], pos[1], pos[2]
    ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
    ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
    return ps


class ArmExecutor:
    """Машинерия исполнения одной руки: traj -> fine -> reached."""
    IDLE, TRAJ, FINE, REACHED, FAILED = "idle", "traj", "fine", "reached", "failed"

    def __init__(self, model):
        self.model = model
        self.state = self.IDLE
        self.traj = None
        self.idx = 0
        self.T_des = np.eye(4)
        self.fine_cycles = 0
        self.last_err = (float("nan"), float("nan"))
        # опорная поза для delta-целей (используется при delta_reference=last_goal):
        self.last_T_des = None
        # Последняя отправленная позиция суставов; повторно публикуется
        # в idle/reached/failed, чтобы контроллер продолжал удерживать позу.
        self.hold_q = None


class ArmIKNode(Node):
    def __init__(self):
        super().__init__("arm_ik_node")
        # ---------------- параметры ----------------
        # TableGuard выключен по умолчанию: геометрия стола должна быть откалибрована
        # именно в base_frame; URDF chain преобразуется в эту рамку через TF2.
        # use_sim_time объявляется автоматически базовым классом Node (ROS 2 Humble);
        # ручное declare_parameter("use_sim_time", ...) роняло ноду с
        # ParameterAlreadyDeclaredException — не добавлять.
        self.declare_parameter("base_frame", "pelvis")
        self.declare_parameter("kinematics_base_frame", "waist_yaw_link")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("joint_state_timeout", 0.5)
        self.declare_parameter("spd_limit", 0.8)       # жёсткий лимит документации
        self.declare_parameter("a_max", 0.4)
        self.declare_parameter("tol_pos", 0.01)        # м
        self.declare_parameter("tol_rot", 0.05)        # рад
        self.declare_parameter("kp", 0.6)              # коррекция недолёта ПД
        self.declare_parameter("ik_rotation_weight", 0.25) # приоритет позиции TCP над ориентацией
        self.declare_parameter("fine_max_cycles", 300) # страховка от бесконечной fine
        self.declare_parameter("h_table", 0.9)         # высота стола над base_frame, м
        self.declare_parameter("table_x0", 0.35)
        self.declare_parameter("table_x1", 1.2)
        self.declare_parameter("table_y_half", 1.0)
        self.declare_parameter("table_margin", 0.02)
        self.declare_parameter("enable_table_guard", False)
        self.declare_parameter("waypoint_lift", 0.25)  # заход сверху, м (FOCUS_TRACKING §Шаг2)
        self.declare_parameter("n_seeds", 12)          # расширенный набор стартов IK
        self.declare_parameter("tcp_offset", "")         # "x,y,z" поверх TF-цепочки (переопределить кончик)
        # Кинематическая модель и пределы суставов берутся только из URDF.
        # --- параметры delta-целей (TwistStamped на /arm_ik/delta_{left,right}) ---
        self.declare_parameter("delta_reference", "measured")  # measured | last_goal
        self.declare_parameter("delta_accumulate", True)       # потоковые delta-команды накапливаются от последней цели
        self.declare_parameter("delta_max_lin", 0.35)          # предохранитель на одну команду, м
        self.declare_parameter("delta_max_rot", 1.2)           # предохранитель на одну команду, рад
        self.declare_parameter("delta_rotation_frame", "base") # base | tcp
        # --- встроенные демо-паблишеры (заменяют отдельные ноды demo_*) ---
        self.declare_parameter("demo_goals", False)   # публиковать PoseStamped-цели
        self.declare_parameter("demo_delta", False)   # публиковать TwistStamped-дельты
        self.declare_parameter("demo_side", "both")   # left | right | both
        self.declare_parameter("demo_x", 0.45)        # м, вперёд
        self.declare_parameter("demo_y_l", 0.25)      # левая рука, y
        self.declare_parameter("demo_y_r", -0.25)     # правая рука, y
        self.declare_parameter("demo_z", 1.0)         # м, вверх (выше стола)
        self.declare_parameter("demo_yaw_deg", -90.0) # ориентация «кисть вниз»
        self.declare_parameter("demo_dx", 0.05)       # delta: смещение вперёд, м
        self.declare_parameter("demo_dz", 0.0)        # delta: смещение вверх, м
        self.declare_parameter("demo_period", 8.0)    # с между публикациями

        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        self.kinematics_base_frame = str(self.get_parameter("kinematics_base_frame").value).strip()
        if not self.base_frame or not self.kinematics_base_frame:
            raise ValueError("base_frame and kinematics_base_frame must be non-empty")
        self.urdf_path = str(self.get_parameter("urdf_path").value).strip()
        # Временные заглушки используются только для создания executor state;
        # IK и публикация команд блокируются до успешной загрузки URDF.
        self.models = {"left": make_default_arm("left"),
                       "right": make_default_arm("right")}
        self._models_ready = False
        for m in self.models.values():
            m.base_frame = self.kinematics_base_frame
        tof = self.get_parameter("tcp_offset").value
        self.tcp_offset_override = None
        if tof:
            v = np.asarray([float(t) for t in str(tof).split(",")], dtype=float)
            if v.shape != (3,) or not np.all(np.isfinite(v)):
                raise ValueError("tcp_offset must contain exactly three finite numbers")
            self.tcp_offset_override = v.copy()

        self.guard = TableGuard(self.get_parameter("h_table").value,
                                self.get_parameter("table_x0").value,
                                self.get_parameter("table_x1").value,
                                self.get_parameter("table_y_half").value,
                                self.get_parameter("table_margin").value) \
            if self.get_parameter("enable_table_guard").value else None

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.joint_state_timeout = float(self.get_parameter("joint_state_timeout").value)
        self.spd = min(float(self.get_parameter("spd_limit").value), 0.8)  # перестраховка
        self.a_max = float(self.get_parameter("a_max").value)
        if not all(math.isfinite(v) for v in
                   (self.rate_hz, self.joint_state_timeout, self.spd, self.a_max)):
            raise ValueError("rate_hz, joint_state_timeout, spd_limit and a_max must be finite")
        if min(self.rate_hz, self.joint_state_timeout, self.spd, self.a_max) <= 0.0:
            raise ValueError("rate_hz, joint_state_timeout, spd_limit and a_max must be positive")
        self.tol_pos = float(self.get_parameter("tol_pos").value)
        self.tol_rot = float(self.get_parameter("tol_rot").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ik_rotation_weight = float(self.get_parameter("ik_rotation_weight").value)
        self.fine_max = int(self.get_parameter("fine_max_cycles").value)
        self.lift = float(self.get_parameter("waypoint_lift").value)
        self.n_seeds = int(self.get_parameter("n_seeds").value)
        self.delta_ref = str(self.get_parameter("delta_reference").value)
        self.delta_accum = bool(self.get_parameter("delta_accumulate").value)
        self.delta_max_lin = float(self.get_parameter("delta_max_lin").value)
        self.delta_max_rot = float(self.get_parameter("delta_max_rot").value)
        self.delta_rotation_frame = str(self.get_parameter("delta_rotation_frame").value).lower()

        numeric_positive = {
            "tol_pos": self.tol_pos, "tol_rot": self.tol_rot, "kp": self.kp,
            "ik_rotation_weight": self.ik_rotation_weight,
            "delta_max_lin": self.delta_max_lin, "delta_max_rot": self.delta_max_rot,
        }
        if any(not math.isfinite(v) or v <= 0.0 for v in numeric_positive.values()):
            raise ValueError("positive finite parameters required: " +
                             ", ".join(numeric_positive))
        if self.fine_max <= 0 or self.n_seeds <= 0 or not math.isfinite(self.lift) or self.lift < 0.0:
            raise ValueError("fine_max_cycles and n_seeds must be positive; waypoint_lift must be >= 0")
        if self.delta_ref not in ("measured", "last_goal"):
            raise ValueError("delta_reference must be 'measured' or 'last_goal'")
        if self.delta_rotation_frame not in ("base", "tcp"):
            raise ValueError("delta_rotation_frame must be 'base' or 'tcp'")

        # ---------------- состояние ----------------
        self.joints = {}
        self._last_joint_state_wall = None
        self._last_joint_state_warning = 0.0
        self._urdf_xml = None
        self._t0 = self.get_clock().now()   # отсчёт для проверки /clock
        self.exec = {s: ArmExecutor(m) for s, m in self.models.items()}
        # Совместимость с вариантами обработчика, читающими fine_max из executor.
        for ex in self.exec.values():
            ex.fine_max = self.fine_max

        # ---------------- ROS интерфейсы ----------------
        self.cmd_pub = None
        if CmdSetMotorPosition is not None:
            self.cmd_pub = self.create_publisher(CmdSetMotorPosition, "/arm/cmd_pos", 10)
        else:
            self.get_logger().warn(
                "bodyctrl_msgs недоступен — IK считается, но команды в /arm/cmd_pos "
                "НЕ публикуются (dry-run). Убедитесь, что окружение робота source'нуто.")
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(PoseStamped, "/arm_ik/goal_left",
                                 lambda m: self.on_goal("left", m), 10)
        self.create_subscription(PoseStamped, "/arm_ik/goal_right",
                                 lambda m: self.on_goal("right", m), 10)
        # delta-цели: шестимерное смещение TCP (TwistStamped: linear=dxyz, angular=rotvec)
        self.create_subscription(TwistStamped, "/arm_ik/delta_left",
                                 lambda m: self.on_delta("left", m), 10)
        self.create_subscription(TwistStamped, "/arm_ik/delta_right",
                                 lambda m: self.on_delta("right", m), 10)
        self.pose_pubs = {
            "left": self.create_publisher(PoseStamped, "/arm_ik/wrist_left_pose", 10),
            "right": self.create_publisher(PoseStamped, "/arm_ik/wrist_right_pose", 10)}
        self.status_pubs = {
            "left": self.create_publisher(String, "/arm_ik/left/status", 10),
            "right": self.create_publisher(String, "/arm_ik/right/status", 10)}

        # TF2 используется только для преобразования целевых поз между рамками.
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        try:  # лимиты суставов по возможности берём из robot_description
            self.create_subscription(
                String, "/robot_description", self.on_robot_desc,
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
        except Exception:
            pass
        # Первая попытка загрузить URDF выполняется сразу, далее повторяется в step().
        self.try_build_urdf_model()

        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)

        # ---------------- встроенные демо-паблишеры ----------------
        # Заменяют отдельные ноды demo_goal_publisher / demo_delta_publisher:
        # одна нода умеет и исполнять цели, и (опционально) сама их публиковать
        # для ручной проверки. Включаются параметрами demo_goals / demo_delta.
        self.demo_goal_pubs = None
        self.demo_delta_pubs = None
        if self.get_parameter("demo_goals").value:
            self.demo_goal_pubs = {
                s: self.create_publisher(PoseStamped, f"/arm_ik/goal_{s}", 10)
                for s in self._demo_sides()}
            self.get_logger().info("demo_goals ON -> публикация PoseStamped целей")
        if self.get_parameter("demo_delta").value:
            self.demo_delta_pubs = {
                s: self.create_publisher(TwistStamped, f"/arm_ik/delta_{s}", 10)
                for s in self._demo_sides()}
            self.get_logger().info("demo_delta ON -> публикация TwistStamped смещений")
        if self.demo_goal_pubs or self.demo_delta_pubs:
            demo_period = float(self.get_parameter("demo_period").value)
            if not math.isfinite(demo_period) or demo_period <= 0.0:
                raise ValueError("demo_period must be finite and positive")
            self.demo_timer = self.create_timer(demo_period, self.demo_tick)

        self.get_logger().info(
            f"arm_ik_node ready | waiting_for_urdf={not self._models_ready} | "
            f"guard={'ON h=%.2f' % self.guard.h if self.guard else 'OFF'} | "
            "custom service=disabled")

    # ------------------------------------------------------------ URDF и время
    def _check_sim_clock(self):
        """Проверка «время из /clock» при работе с симулятором.

        * если use_sim_time=true — убеждаемся, что в системе есть источник
          времени симуляции (топик /clock) и нодa реально по нему идёт;
        * если use_sim_time=false, но /clock публикуется (симуляция запущена),
          предупреждаем: штампы целей/телеметрии и TF не сойдутся по времени.
        Проверка выполняется один раз после старта (в первом step()).
        """
        if (self.get_clock().now() - self._t0).nanoseconds / 1e9 < 3.0:
            return                      # даём симулятору время поднять /clock
        self._clock_checked = True
        try:
            names = dict(self.get_topic_names_and_types())
        except Exception:
            return
        has_clock = "/clock" in names
        sim = bool(self.get_parameter("use_sim_time").value) \
            if self.has_parameter("use_sim_time") else False
        if sim:
            if not has_clock:
                self.get_logger().warn(
                    "use_sim_time:=true, но топик /clock НЕ публикуется — "
                    "часы симуляции недоступны (запустите симулятор/ros2_sim "
                    "или переключите use_sim_time:=false)")
            else:
                self.get_logger().info(
                    "время берётся из /clock симулятора (use_sim_time=true)")
        elif has_clock:
            self.get_logger().warn(
                "обнаружен /clock (идёт симуляция), но use_sim_time=false — "
                "штампы /joint_states, /tf и целей будут в разных шкалах; "
                "рекомендуется ros2 launch ... use_sim_time:=true")

    def on_robot_desc(self, msg):
        """Receive the transient-local robot_description string."""
        xml = str(msg.data).strip()
        if xml:
            self._urdf_xml = xml
            self.try_build_urdf_model()

    def try_build_urdf_model(self):
        """Load exact arm chains and joint limits from a file or robot_description.

        TF alone is deliberately not accepted as a kinematic source: it cannot
        recover the original joint axes, joint types or hard position limits.
        """
        if self._models_ready:
            return True

        candidates = []
        if self.urdf_path:
            try:
                with open(self.urdf_path, "r", encoding="utf-8") as stream:
                    candidates.append(("file", stream.read()))
            except OSError as exc:
                self._log_urdf_error("file:" + str(exc),
                                     f"URDF file unavailable ({self.urdf_path}): {exc}")

        if self._urdf_xml:
            candidates.append(("/robot_description", self._urdf_xml))

        if not candidates:
            if not getattr(self, "_urdf_missing_warned", False):
                self.get_logger().warning(
                    "URDF is not available. Set urdf_path or publish transient-local "
                    "/robot_description (std_msgs/msg/String). IK remains disabled.")
                self._urdf_missing_warned = True
            return False

        from arm_ik.urdf_model import build_models_from_urdf
        for source, xml in candidates:
            try:
                models = build_models_from_urdf(
                    xml, base_link=self.kinematics_base_frame)
                if set(models) != {"left", "right"}:
                    raise ValueError("URDF parser did not produce both arm models")

                all_motor_ids = []
                for side, model in models.items():
                    if len(model.joints) != 7:
                        raise ValueError(f"{side} arm must have exactly 7 movable joints")
                    if len(set(model.ids)) != 7 or any(mid <= 0 for mid in model.ids):
                        raise ValueError(f"{side} arm has invalid or duplicate motor IDs")
                    model.base_frame = self.kinematics_base_frame
                    if self.tcp_offset_override is not None:
                        model.tcp_offset = self.tcp_offset_override.copy()
                    all_motor_ids.extend(model.ids)
                if len(set(all_motor_ids)) != 14:
                    raise ValueError("left/right arm motor IDs must be globally unique")

                self.models = models
                for side, model in models.items():
                    self.exec[side].model = model
                self._models_ready = True
                self.get_logger().info(
                    f"Exact IK models loaded from {source}: " + "; ".join(
                        f"{side}: {model.names[0]}..{model.names[-1]} -> {model.tip_link}, "
                        f"motor_ids={model.ids}"
                        for side, model in models.items()))
                return True
            except Exception as exc:
                self._log_urdf_error(source + ":" + str(exc),
                                     f"Failed to build IK models from {source}: {exc}")

        return False

    def _log_urdf_error(self, key, message):
        if getattr(self, "_last_urdf_error", None) != key:
            self.get_logger().error(message)
            self._last_urdf_error = key

    def _demo_sides(self):
        side = str(self.get_parameter("demo_side").value)
        if side == "both":
            return ["left", "right"]
        if side in ("left", "right"):
            return [side]
        raise ValueError("demo_side must be 'left', 'right' or 'both'")

    def demo_tick(self):
        """Периодическая публикация демо-целей/дельт (внутренний таймер ноды)."""
        g = lambda n: float(self.get_parameter(n).value)
        now = self.get_clock().now().to_msg()
        if self.demo_goal_pubs:
            yaw = math.radians(g("demo_yaw_deg"))
            q = Rotation.from_euler("xyz", [0.0, 0.0, yaw]).as_quat()
            ys = {"left": g("demo_y_l"), "right": g("demo_y_r")}
            for s, pub in self.demo_goal_pubs.items():
                ps = PoseStamped()
                ps.header.frame_id = self.base_frame
                ps.header.stamp = now
                ps.pose.position.x, ps.pose.position.y = g("demo_x"), ys[s]
                ps.pose.position.z = g("demo_z")
                ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
                ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
                pub.publish(ps)
            self.get_logger().info("demo: опубликованы абсолютные цели (PoseStamped)")
        if self.demo_delta_pubs:
            for s, pub in self.demo_delta_pubs.items():
                ts = TwistStamped()
                ts.header.frame_id = self.base_frame
                ts.header.stamp = now
                ts.twist.linear.x, ts.twist.linear.z = g("demo_dx"), g("demo_dz")
                pub.publish(ts)
            self.get_logger().info(
                f"demo: delta dp=(+{g('demo_dx'):.3f},0,+{g('demo_dz'):.3f}) м")

    # ------------------------------------------------------------ helpers
    def on_js(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if not all(np.isfinite(float(v)) for v in values.values()):
            return
        if not values:
            return
        self.joints.update(values)
        self._last_joint_state_wall = time.monotonic()
        self._joint_state_received = True

    def q_meas(self, model):
        missing = [name for name in model.names if name not in self.joints]
        if missing:
            raise RuntimeError("нет измерений /joint_states для: " + ", ".join(missing))
        q = np.asarray([self.joints[n] for n in model.names], dtype=float)
        if not np.all(np.isfinite(q)):
            raise RuntimeError("некорректные значения /joint_states")
        return q

    def make_cmd(self, model, q_target, spd=None):
        """Пачка команд моторов руки. Формат — как в docs/INTERFACES.md и hello_world:
        CmdSetMotorPosition.cmds = [SetMotorPosition(name=ID, pos, spd, cur), ...]"""
        spd = self.spd if spd is None else spd
        msg = CmdSetMotorPosition()
        msg.cmds = [SetMotorPosition(name=int(mid), pos=float(q), spd=float(spd), cur=1.0)
                    for mid, q in zip(model.ids, q_target)]
        return msg

    def to_base_T(self, goal: PoseStamped):
        """PoseStamped в любой TF-рамке -> матрица 4x4 в base_frame."""
        fid = goal.header.frame_id
        if fid in ("", self.base_frame):
            return pose_to_T(goal.pose)
        stamp = (Time.from_msg(goal.header.stamp)
                 if goal.header.stamp.sec or goal.header.stamp.nanosec
                 else Time())
        tfm = self.tf_buffer.lookup_transform(self.base_frame, fid, stamp,
                                              timeout=Duration(seconds=0.5))
        t = tfm.transform.translation
        r = tfm.transform.rotation
        T_bc = np.eye(4)
        T_bc[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T_bc[:3, 3] = [t.x, t.y, t.z]
        return T_bc @ pose_to_T(goal.pose)

    def seed_poses(self, model, q_now):
        """Расширенный детерминированный набор IK-стартов в пределах URDF.

        Первый seed — измеренная конфигурация (минимум движения и быстрый путь).
        Остальные покрывают альтернативные плечевые/локтевые конфигурации, чтобы
        не считать локальный минимум текущей позы границей рабочего пространства.
        """
        q_now = np.asarray(q_now, dtype=float)
        seeds = [clip_to_limits(q_now, model)]
        home = clip_to_limits(np.zeros(7), model, margin=0.05)
        reach = np.array([-0.4, 0.3, 0.0, -0.9, 0.0, 0.2, 0.0])
        bent = np.array([-0.8, 0.5, 0.4, -1.6, 0.3, 0.4, 0.0])
        if model.side == "left":
            reach[1] = -reach[1]
            bent[1] = -bent[1]
        seeds.extend((home,
                      clip_to_limits(reach, model, margin=0.03),
                      clip_to_limits(bent, model, margin=0.03)))

        # Парные возмущения меняют конфигурацию плеча/локтя, сохраняя все
        # реальные суставные пределы. Они используются только если measured
        # seed не сошёлся, поэтому расширение поиска не замедляет обычные цели.
        patterns = (
            (0.65, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (-0.65, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.55, 0.0, -0.55, 0.0, 0.0, 0.0),
            (0.0, -0.55, 0.0, -0.55, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.65, -0.45, 0.0, 0.0, 0.0),
            (0.0, 0.0, -0.65, -0.45, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.65, 0.55, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.65, -0.55, 0.0, 0.0),
        )
        for offset in patterns:
            seeds.append(clip_to_limits(q_now + np.asarray(offset), model, margin=0.02))

        # После clipping некоторые стартовые конфигурации могут совпасть.
        unique = []
        for q in seeds:
            if not any(np.linalg.norm(q - prev) < 1e-4 for prev in unique):
                unique.append(q)
        return unique[:max(1, self.n_seeds)]

    # ------------------------------------------------------------ goals
    def _lookup_T(self, target_frame, source_frame):
        """Матрица T_target_source по актуальному TF2."""
        if target_frame == source_frame:
            return np.eye(4)
        tfm = self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time(),
            timeout=Duration(seconds=0.5))
        t, r = tfm.transform.translation, tfm.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def set_goal(self, side, T_des, position_only=False):
        """Общий конвейер: IK -> траектория (с защитой стола) -> исполнение."""
        if not self._models_ready:
            self.get_logger().warn(f"[{side}] IK rejected: URDF model is not ready")
            return False
        ex = self.exec[side]
        model = self.models[side]
        if not all(name in self.joints for name in model.names):
            self.get_logger().warn(f"[{side}] IK rejected: /joint_states has not supplied all 7 arm joints")
            return False
        T_des = np.asarray(T_des, dtype=float).copy()
        if T_des.shape != (4, 4) or not np.all(np.isfinite(T_des)):
            self.get_logger().error(f"[{side}] goal rejected: pose matrix is invalid")
            return False
        if self.guard is not None:
            T_des[:3, 3] = self.guard.clamp_target_above_table(T_des[:3, 3])
        try:
            T_model_base = self._lookup_T(
                self.kinematics_base_frame, self.base_frame)
            T_des_model = T_model_base @ T_des
            q_now = self.q_meas(model)
        except Exception as e:
            self.get_logger().error("[%s] цель отклонена: нет TF/joint state: %s" %
                                    (side, e))
            return False
        ex.T_des = T_des_model

        T_base_model = np.linalg.inv(T_model_base)
        barrier = None
        if self.guard is not None:
            barrier = lambda q: self.guard.table_cost_gradient(
                model, q, k=30.0, T_guard_model=T_base_model)

        # 1) IK: multi-seed DLS с барьером стола
        seeds = self.seed_poses(model, q_now)
        task_mask = ([True, True, True, False, False, False]
                     if position_only else [True] * 6)
        q_sol, ok, epos, erot = ik_multi_seed(
            model, T_des_model, seeds, max_iter=350, tol_pos=self.tol_pos * 0.5,
            tol_rot=self.tol_rot * 0.5, barrier=barrier,
            task_weights=(1.0, self.ik_rotation_weight), task_mask=task_mask)

        dt = 1.0 / self.rate_hz
        if ok and self.guard is not None:
            traj = joint_space_profile(q_now, q_sol, self.spd, self.a_max, dt)
            bad = any(self.guard.collides_with_table(
                model, qq, T_guard_model=T_base_model) for qq in traj)
        else:
            bad = False

        if ok and not bad:
            ex.traj = joint_space_profile(q_now, q_sol, self.spd, self.a_max, dt)
            ex.idx, ex.state, ex.fine_cycles = 0, ArmExecutor.TRAJ, 0
            ex.last_T_des = np.linalg.inv(T_model_base) @ T_des_model
            self.get_logger().info(
                f"[{side}] IK решена: {len(ex.traj)} точек (~{(len(ex.traj)-1)/self.rate_hz:.1f} c)")
            return True

        # 2) план Б: заход через waypoint строго над столом (две IK-фазы)
        if self.guard is not None and self.lift > 0.0:
            # Настоящий waypoint: сначала поднять TCP, затем идти к финальной
            # цели. Оба суставных сегмента проверяются сэмплированием звеньев.
            wp = T_des_model.copy()
            wp_base = T_base_model @ wp
            wp_base[2, 3] += self.lift
            wp = T_model_base @ wp_base
            q_wp, ok1, _, _ = ik_multi_seed(
                model, wp, seeds, max_iter=300, barrier=barrier,
                task_weights=(1.0, self.ik_rotation_weight), task_mask=task_mask)
            if ok1:
                q_fin, ok2, _, _ = ik_multi_seed(
                    model, T_des_model, [q_wp] + seeds, max_iter=350,
                    barrier=barrier,
                    task_weights=(1.0, self.ik_rotation_weight), task_mask=task_mask)
                if ok2:
                    t1 = joint_space_profile(q_now, q_wp, self.spd, self.a_max, dt)
                    t2 = joint_space_profile(q_wp, q_fin, self.spd, self.a_max, dt)
                    route = np.vstack([t1[:-1], t2])
                    if not any(self.guard.collides_with_table(
                            model, qq, T_guard_model=T_base_model) for qq in route):
                        ex.traj = route
                        ex.idx, ex.state, ex.fine_cycles = 0, ArmExecutor.TRAJ, 0
                        ex.last_T_des = np.linalg.inv(T_model_base) @ T_des_model
                        self.get_logger().info(
                            f"[{side}] IK через проверенный waypoint над столом "
                            f"({len(ex.traj)} точек)")
                        return True

        ex.state, ex.traj = ArmExecutor.FAILED, None
        ex.last_err = (epos, erot)
        self.get_logger().error(
            f"[{side}] IK не сошлась (err pos={epos*100:.1f} см rot={erot:.2f} рад) — удержание")
        return False

    def on_goal(self, side, goal: PoseStamped):
        """Абсолютная шестимерная цель TCP (PoseStamped)."""
        if not self._models_ready:
            self.get_logger().warn(f"[{side}] goal rejected: waiting for /robot_description")
            return
        try:
            T_des = self.to_base_T(goal)
        except Exception as e:  # TF ещё не доступен
            self.get_logger().error(f"[{side}] TF {self.base_frame}<-{goal.header.frame_id}: {e}")
            return
        self.set_goal(side, T_des)

    def tcp_pose_T(self, side):
        """Измеренная TCP-поза, возвращаемая в planning base_frame (base_frame)."""
        model = self.models[side]
        T_model_tcp = fk(model, self.q_meas(model))[0]
        T_base_model = self._lookup_T(self.base_frame, self.kinematics_base_frame)
        return T_base_model @ T_model_tcp

    def on_delta(self, side, msg: TwistStamped):
        """Относительная цель: смещение TCP на 6-мерный вектор (TwistStamped).

        linear = dxyz (м), angular = rotation-vector (ось*угол, рад), не угловая скорость; оба в base_frame.
        Опорная поза (параметр delta_reference):
          'measured'  — текущая ИЗМЕРЕННАЯ поза TCP (FK от энкодеров);
                        повторная публикация той же дельты = «ещё раз на столько же»
                        относительно фактического положения кисти;
          'last_goal' — последняя поставленная абсолютная цель (идемпотентно).
        delta_accumulate=true — дельты суммируются от опорной точки первой дельты
        серии (джойстик-режим: каждая новая команда двигает дальше).
        """
        if not self._models_ready:
            self.get_logger().warn(f"[{side}] delta rejected: URDF model is not ready")
            return
        ex = self.exec[side]
        fid = msg.header.frame_id
        dpos = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        drot = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        if fid not in ("", self.base_frame):
            try:
                stamp = (Time.from_msg(msg.header.stamp)
                         if msg.header.stamp.sec or msg.header.stamp.nanosec
                         else Time())
                tfm = self.tf_buffer.lookup_transform(
                    self.base_frame, fid, stamp,
                    timeout=Duration(seconds=0.2))
                rot = tfm.transform.rotation
                R_frame_base = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
                dpos = R_frame_base @ dpos
                drot = R_frame_base @ drot
            except Exception as exc:
                self.get_logger().warn(
                    f"[{side}] delta rejected: TF {self.base_frame}<-{fid} unavailable: {exc}")
                return
        if not all(name in self.joints for name in self.models[side].names):
            self.get_logger().warn(f"[{side}] delta rejected: /joint_states is incomplete")
            return

        # опорная поза выбирается ДО валидации и не меняется при отклонении дельты
        use_last = (self.delta_ref == "last_goal" and ex.last_T_des is not None) or \
                   (self.delta_accum and ex.last_T_des is not None
                    and ex.state in (ArmExecutor.TRAJ, ArmExecutor.FINE, ArmExecutor.REACHED))
        if use_last:
            T_ref = ex.last_T_des
        else:
            T_ref = self.tcp_pose_T(side)

        # предохранители: одна дельта не должна быть гигантской
        nl, nr = np.linalg.norm(dpos), np.linalg.norm(drot)
        if nl > self.delta_max_lin or nr > self.delta_max_rot:
            self.get_logger().warn(
                f"[{side}] delta слишком большая (|dp|={nl:.3f} м, |dr|={nr:.3f} рад; "
                f"лимиты {self.delta_max_lin}/{self.delta_max_rot}) — отклонена")
            return
        if nl < 1e-9 and nr < 1e-9:
            return  # нулевая дельта — ничего делать не нужно

        # angular задаёт конечный поворот за команду (rotation vector), а не скорость.
        if self.delta_rotation_frame == "tcp":
            if fid not in ("", self.base_frame):
                self.get_logger().warning(
                    "delta_rotation_frame=tcp требует header.frame_id пустой или base_frame")
                return
            drot = T_ref[:3, :3] @ drot
        elif self.delta_rotation_frame != "base":
            self.get_logger().error(
                "delta_rotation_frame должен быть 'base' или 'tcp'; delta отклонена")
            return
        T_des = delta_to_T(T_ref, dpos, drot, base_ref=True)
        self.get_logger().info(
            f"[{side}] delta-цель: dp=({dpos[0]:+.3f},{dpos[1]:+.3f},{dpos[2]:+.3f}) м, "
            f"dr=({drot[0]:+.3f},{drot[1]:+.3f},{drot[2]:+.3f}) рад "
            f"[ref={'last_goal' if use_last else 'measured'}, rot_frame={self.delta_rotation_frame}]")
        self.set_goal(side, T_des, position_only=(nl > 1e-9 and nr < 1e-9))

    # ------------------------------------------------------------ control loop
    def step(self):
        now = self.get_clock().now().to_msg()
        # Попытка загрузить точную кинематику из файла или /robot_description.
        self.try_build_urdf_model()
        if not getattr(self, "_clock_checked", False):
            self._check_sim_clock()
        if not self._models_ready:
            return
        # Watchdog использует monotonic wall time, чтобы работать даже при
        # остановленных/отсутствующих ROS simulation clocks.
        last_js = self._last_joint_state_wall
        if last_js is None or time.monotonic() - last_js > self.joint_state_timeout:
            for ex in self.exec.values():
                if ex.state in (ArmExecutor.TRAJ, ArmExecutor.FINE):
                    ex.state = ArmExecutor.FAILED
                ex.traj = None
                ex.idx = 0
                ex.hold_q = None
            wall_now = time.monotonic()
            if wall_now - self._last_joint_state_warning > 2.0:
                self.get_logger().error(
                    f"/joint_states stale or missing (> {self.joint_state_timeout:.2f}s); "
                    "motor command publishing suspended")
                self._last_joint_state_warning = wall_now
            return
        for side, model in self.models.items():
            ex = self.exec[side]
            if not all(name in self.joints for name in model.names):
                continue
            q_meas = self.q_meas(model)
            if ex.hold_q is None:
                # До первой команды удерживаем фактически измеренную позу.
                ex.hold_q = q_meas.copy()

            # телеметрия: текущая поза TCP (FK от энкодеров)
            ps = T_to_posemsg(fk(model, q_meas)[0])
            ps.header.frame_id = self.base_frame
            ps.header.stamp = now
            self.pose_pubs[side].publish(ps)

            if ex.traj is not None and ex.idx < len(ex.traj):
                # Re-check against the current torso yaw transform immediately
                # before publishing: the torso may have moved since planning.
                q_command = np.asarray(ex.traj[ex.idx], dtype=float).copy()
                if self.guard is not None:
                    try:
                        T_guard_model = self._lookup_T(
                            self.base_frame, self.kinematics_base_frame)
                        blocked = self.guard.collides_with_table(
                            model, q_command, T_guard_model=T_guard_model)
                    except Exception as exc:
                        blocked = True
                        self.get_logger().error(
                            f"[{side}] table-frame TF unavailable; stopping arm command: {exc}")
                    if blocked:
                        ex.state = ArmExecutor.FAILED
                        ex.traj = None
                        ex.idx = 0
                        self.get_logger().error(
                            f"[{side}] trajectory point blocked by table guard; "
                            "holding the last safe command")
                        q_command = None
                if q_command is not None:
                    if self.cmd_pub is not None:
                        self.cmd_pub.publish(self.make_cmd(model, q_command))
                    ex.hold_q = q_command
                    ex.idx += 1
            elif ex.state == ArmExecutor.TRAJ:
                # Основной профиль закончился. Следующий профиль будет
                # построен fine-контуром ниже.
                ex.state = ArmExecutor.FINE
                ex.traj = None
                ex.idx = 0
            else:
                # Fine-профиль полностью исполнен: разрешаем контроллеру
                # построить следующую коррекцию от новых измеренных углов.
                if ex.state == ArmExecutor.FINE and ex.traj is not None:
                    ex.traj = None
                    ex.idx = 0
                # Между целями и после завершения/ошибки продолжаем посылать
                # последнюю командную позу с частотой основного control loop.
                if self.cmd_pub is not None and ex.hold_q is not None:
                    self.cmd_pub.publish(self.make_cmd(model, ex.hold_q))

            if ex.state == ArmExecutor.FINE:
                # Замкнутый контур по /joint_states. Не перезаписываем активный
                # fine-профиль на каждом тике: сначала отправляем все его точки,
                # затем строим следующую коррекцию по свежим измерениям.
                e = pose_error(fk(model, q_meas)[0], ex.T_des)
                ex.last_err = (float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:])))
                if ex.last_err[0] < self.tol_pos and ex.last_err[1] < self.tol_rot:
                    ex.state = ArmExecutor.REACHED
                    ex.traj = None
                    self.get_logger().info(
                        f"[{side}] цель достигнута: pos err {ex.last_err[0]*100:.1f} см, "
                        f"rot err {ex.last_err[1]:.3f} рад")
                elif ex.fine_cycles >= self.fine_max:
                    ex.state = ArmExecutor.FAILED
                    ex.traj = None
                    self.get_logger().warn(f"[{side}] fine-фаза исчерпала {ex.fine_max} циклов")
                elif ex.traj is None:
                    J = geometric_jacobian(model, q_meas)
                    A = J @ J.T + 0.05 ** 2 * np.eye(6)
                    dq = J.T @ np.linalg.solve(A, e)
                    dq = np.clip(dq, -0.1, 0.1)          # ≤ 0.1 рад/цикл (FOCUS_TRACKING §3)
                    q_new = clip_to_limits(q_meas + self.kp * dq, model, margin=1e-4)
                    ex.traj = joint_space_profile(q_meas, q_new,
                                                  min(self.spd, 0.3), self.a_max,
                                                  1.0 / self.rate_hz)
                    ex.idx = 0
                    ex.fine_cycles += 1

            st = {"side": side, "state": ex.state,
                  "err_pos_m": round(ex.last_err[0], 4) if ex.last_err[0] == ex.last_err[0] else None,
                  "err_rot_rad": round(ex.last_err[1], 4) if ex.last_err[1] == ex.last_err[1] else None}
            msg = String()
            msg.data = json.dumps(st)
            self.status_pubs[side].publish(msg)


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
