# -*- coding: utf-8 -*-
"""ROS 2 узел обратной кинематики обеих рук TienKung 2.0 Pro (7 DOF на руку).

Интерфейсы:
  ВХОД (два способа задать позу кисти):
    * /arm_ik/goal_left, /arm_ik/goal_right   (geometry_msgs/PoseStamped)
        АБСОЛЮТНАЯ шестимерная поза TCP кисти (позиция + ориентация).
        Рамка header.frame_id может быть любой из дерева TF
        (pelvis, camera_head_link, ...) — узел сам перенесёт её в pelvis через TF2.
        Рамка pelvis/пустая — без переноса («ориентация относительно корпуса»).
    * /arm_ik/delta_left, /arm_ik/delta_right (geometry_msgs/TwistStamped)
        ОТНОСИТЕЛЬНОЕ шестимерное смещение: linear = dxyz (м),
        angular = ось*угол (рад) — малое вращение в системе base_frame (pelvis).
        Цель = текущая измеренная поза TCP (FK по /joint_states) + смещение.
        Опции поведения — параметры delta_reference / delta_accumulate /
        delta_max_* (подробности в README пакета).
    * ~/solve                                  (arm_ik/srv/SolveIK)
        решить IK без движения (для планирования/отладки верхним уровнем).
        Сервис появляется только если пакет собран с rosidl-генерацией srv
        (см. README); без него узел полностью работает от топиков целей.
  ВЫХОД:
    * /arm/cmd_pos            (bodyctrl_msgs/CmdSetMotorPosition) — команды моторов
                              11..17 (левая) и 21..27 (правая), spd <= 0.8 рад/с.
    * /arm_ik/wrist_{left,right}_pose (PoseStamped, рамка pelvis) — текущая поза
                              TCP по FK от /joint_states (видно в RViz, полезно
                              верхнему уровню как «где сейчас кисть»).
    * /arm_ik/{left,right}/status (std_msgs/String, JSON) — фаза контура:
                              idle | traj | fine | reached | failed + ошибки.

Конвейер исполнения цели (подробно — README пакета):
  1. TF-перенос цели в pelvis, клампинг выше стола (TableGuard);
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
import os
import math

from arm_ik._math_compat import np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
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

from arm_ik.kinematics import (fk, fk_tcp, geometric_jacobian, ik_dls,
                               ik_multi_seed, clip_to_limits, load_arm_model,
                               pose_error, delta_to_T, pose_to_T as mat_from_posquat,
                               _tmat)
from arm_ik.collision import TableGuard
from arm_ik.trajectory import joint_space_profile

try:  # srv-кодген включается только если пакет собран с rosidl (см. README)
    from arm_ik.srv import SolveIK
    HAVE_SRV = True
except ImportError:
    HAVE_SRV = False


def pose_to_T(msg_pose):
    p = msg_pose.position
    q = msg_pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
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


class ArmIKNode(Node):
    def __init__(self):
        super().__init__("arm_ik_node")
        # ---------------- параметры ----------------
        # use_sim_time объявляется автоматически базовым классом Node (ROS 2 Humble);
        # ручное declare_parameter("use_sim_time", ...) роняло ноду с
        # ParameterAlreadyDeclaredException — не добавлять.
        self.declare_parameter("model_path", "")       # config/arms.json (из URDF)
        self.declare_parameter("base_frame", "pelvis")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("spd_limit", 0.8)       # жёсткий лимит документации
        self.declare_parameter("a_max", 0.4)
        self.declare_parameter("tol_pos", 0.01)        # м
        self.declare_parameter("tol_rot", 0.05)        # рад
        self.declare_parameter("kp", 0.6)              # коррекция недолёта ПД
        self.declare_parameter("fine_max_cycles", 300) # страховка от бесконечной fine
        self.declare_parameter("h_table", 0.9)         # высота стола над pelvis, м
        self.declare_parameter("table_x0", 0.35)
        self.declare_parameter("table_x1", 1.2)
        self.declare_parameter("table_y_half", 1.0)
        self.declare_parameter("table_margin", 0.02)
        self.declare_parameter("enable_table_guard", True)
        self.declare_parameter("waypoint_lift", 0.25)  # заход сверху, м (FOCUS_TRACKING §Шаг2)
        self.declare_parameter("n_seeds", 4)           # альтернативных стартов IK
        self.declare_parameter("tcp_offset", "")         # "x,y,z" поверх arms.json (переопределить кончик)
        # --- источник дерева звеньев: /tf + /tf_static симулятора (по умолчанию) ---
        self.declare_parameter("tree_source", "tf")     # tf | file
        self.declare_parameter("tf_wait_timeout", 10.0) # с ожидания первого полного дерева TF
        self.declare_parameter("tf_refresh_period", 5.0) # с периодической пересборки цепочки из TF
        # --- параметры delta-целей (TwistStamped на /arm_ik/delta_{left,right}) ---
        self.declare_parameter("delta_reference", "measured")  # measured | last_goal
        self.declare_parameter("delta_accumulate", False)      # суммировать ли дельты между собой
        self.declare_parameter("delta_max_lin", 0.15)          # предохранитель, м
        self.declare_parameter("delta_max_rot", 0.6)           # предохранитель, рад
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

        self.base_frame = self.get_parameter("base_frame").value
        # ---------------- источник дерева звеньев для IK ----------------
        # По умолчанию ("tf") цепочка звеньев строится из топиков симулятора
        # /tf и /tf_static (см. arm_ik.tree_from_tf); "file" — прежнее
        # поведение (config/arms.json, сгенерированный из URDF).
        self.tree_source = str(self.get_parameter("tree_source").value).lower()
        self.tf_wait_timeout = float(self.get_parameter("tf_wait_timeout").value)
        self.tf_refresh = float(self.get_parameter("tf_refresh_period").value)
        self._model_path = None
        if self.tree_source != "tf":
            mp = self.get_parameter("model_path").value or None
            if not mp:
                # авто-поиск: рядом с пакетом (config/arms.json) и в share/ установках
                from arm_ik.kinematics import _candidate_config_paths
                here = os.path.dirname(os.path.abspath(__file__))          # .../arm_ik/nodes
                pkg_root = os.path.dirname(os.path.dirname(here))           # корень пакета
                cand = [os.path.join(pkg_root, "config", "arms.json")]
                try:
                    from ament_index_python.packages import get_package_share_directory
                    cand.append(os.path.join(get_package_share_directory("arm_ik"),
                                             "config", "arms.json"))
                except Exception:
                    pass
                cand += _candidate_config_paths()
                mp = next((c for c in cand if c and os.path.exists(c)), None)
                if mp:
                    self.get_logger().info(f"модель IK autoload: {mp}")
            self._model_path = mp
            self.models = {"left": load_arm_model(mp, "left"),
                           "right": load_arm_model(mp, "right")}
        else:
            # TF-режим: стартовые модели — встроенный шаблон (нужны сразу для
            # FK/телеметрии), затем перезаменяются деревом из /tf+/tf_static,
            # как только оно появится в буфере.
            self.models = {"left": load_arm_model(None, "left"),
                           "right": load_arm_model(None, "right")}
            self._tf_models_ready = False
        for m in self.models.values():
            m.base_frame = self.base_frame
        tof = self.get_parameter("tcp_offset").value
        if tof:
            v = [float(t) for t in str(tof).split(",")]
            for m in self.models.values():
                m.tcp_offset = np.array(v)

        self.guard = TableGuard(self.get_parameter("h_table").value,
                                self.get_parameter("table_x0").value,
                                self.get_parameter("table_x1").value,
                                self.get_parameter("table_y_half").value,
                                self.get_parameter("table_margin").value) \
            if self.get_parameter("enable_table_guard").value else None

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.spd = min(float(self.get_parameter("spd_limit").value), 0.8)  # перестраховка
        self.a_max = float(self.get_parameter("a_max").value)
        self.tol_pos = float(self.get_parameter("tol_pos").value)
        self.tol_rot = float(self.get_parameter("tol_rot").value)
        self.kp = float(self.get_parameter("kp").value)
        self.fine_max = int(self.get_parameter("fine_max_cycles").value)
        self.lift = float(self.get_parameter("waypoint_lift").value)
        self.n_seeds = int(self.get_parameter("n_seeds").value)
        self.delta_ref = str(self.get_parameter("delta_reference").value)
        self.delta_accum = bool(self.get_parameter("delta_accumulate").value)
        self.delta_max_lin = float(self.get_parameter("delta_max_lin").value)
        self.delta_max_rot = float(self.get_parameter("delta_max_rot").value)

        # ---------------- состояние ----------------
        self.joints = {}
        self._t0 = self.get_clock().now()   # отсчёт для проверки /clock
        self.exec = {s: ArmExecutor(m) for s, m in self.models.items()}
        self._tf_last_try = None      # когда последний раз просили дерево TF
        self._urdf_xml = None         # robot_description (для лимитов), если придёт

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

        # TF2: перенос произвольной рамки цели в base_frame + источник дерева
        # звеньев для IK (топики симулятора /tf и /tf_static)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        if self.tree_source == "tf":
            try:  # лимиты суставов по возможности берём из robot_description
                from std_msgs.msg import String as _StrMsg
                self.create_subscription(
                    _StrMsg, "/robot_description", self.on_robot_desc,
                    rclpy.qos.QoSProfile(
                        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                        history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1,
                        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
            except Exception:
                pass
            # первая попытка построить дерево — сразу, далее по таймеру step
            self.try_build_tree_from_tf()

        cbg = ReentrantCallbackGroup()
        if HAVE_SRV:
            self.create_service(SolveIK, "~/solve", self.solve_srv, callback_group=cbg)
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
            self.demo_timer = self.create_timer(
                float(self.get_parameter("demo_period").value), self.demo_tick)

        self.get_logger().info(
            f"arm_ik_node ready | tips={[m.tip_link for m in self.models.values()]} | "
            f"tree={self.tree_source}{' (ждём /tf, /tf_static)' if self.tree_source == 'tf' and not getattr(self, '_tf_models_ready', True) else ''} | "
            f"guard={'ON h=%.2f' % self.guard.h if self.guard else 'OFF'} | "
            f"srv={'on' if HAVE_SRV else 'off (нет сгенерированного arm_ik.srv)'}")

    # ------------------------------------------------------------ дерево из TF
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
        """robot_description (URDF-XML) — источник точных лимитов суставов."""
        try:
            self._urdf_xml = str(msg.data)
        except Exception:
            pass

    def _tf_snapshot(self):
        """([static TFMessage], [dynamic TFMessage]) из буфера TF2.

        Берём и static-рёбра (/tf_static — постоянные origin'ы), и динамические
        (/tf — активные суставные соединения симулятора).
        """
        static_msgs, dyn_msgs = [], []
        try:
            st, dy = self.tf_buffer.all_as_listofmessages()
            static_msgs, dyn_msgs = list(st), list(dy)
        except Exception:
            try:  # старые версии tf2_ros: только all_frames_as_string/lookup
                st, dy = self.tf_buffer.all_as_listofmessages() \
                    if hasattr(self.tf_buffer, "all_as_listofmessages") else ([], [])
                static_msgs, dyn_msgs = list(st), list(dy)
            except Exception as e:
                self.get_logger().debug(f"tf snapshot недоступен: {e}")
        return static_msgs, dyn_msgs

    def try_build_tree_from_tf(self, force=False):
        """Пересобрать кинематические модели обеих рук из /tf + /tf_static.

        Вызывается при старте и периодически (tf_refresh_period) из step():
        пока дерево не получено, нода живёт на шаблонной модели; как только
        симулятор опубликовал полные /tf+/tf_static — цепочка звеньев
        пересобирается из TF и executor'ы переключаются на новые модели.
        """
        if self.tree_source != "tf":
            return False
        now = self.get_clock().now()
        if not force:
            last = self._tf_last_try
            period = (0.0 if not getattr(self, "_tf_models_ready", False)
                      else self.tf_refresh)
            if last is not None and (now - last).nanoseconds / 1e9 < period:
                return False
        self._tf_last_try = now

        static_msgs, dyn_msgs = self._tf_snapshot()
        if not static_msgs and not dyn_msgs:
            waited = (now - self._t0).nanoseconds / 1e9
            if not getattr(self, "_tf_warned", False):
                self.get_logger().warn(
                    "дерево звеньев ещё не получено из /tf, /tf_static — "
                    "работаю на шаблонной модели (проверьте robot_state_publisher "
                    "и use_sim_time)")
                self._tf_warned = True
            if waited > self.tf_wait_timeout and \
                    not getattr(self, "_tf_timeout_warned", False):
                self._tf_timeout_warned = True
                self.get_logger().error(
                    f"за {self.tf_wait_timeout:.0f} с дерево из /tf+/tf_static не "
                    "появилось; для запуска без симулятора используйте "
                    "-p tree_source:=file -p model_path:=<arms.json>")
            return False

        from arm_ik.tree_from_tf import build_models_from_tf
        tcp_ov = None
        tof = self.get_parameter("tcp_offset").value
        if tof:
            v = [float(t) for t in str(tof).split(",")]
            tcp_ov = {"left": v, "right": v}
        try:
            models = build_models_from_tf(
                static_msgs, dyn_msgs, sorted(self.joints), self.base_frame,
                urdf_xml=self._urdf_xml, tcp_offsets=tcp_ov,
                logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"построение дерева IK из /tf failed: {e}")
            return False
        if len(models) < 2:
            if not getattr(self, "_tf_models_ready", False):
                self.get_logger().warn(
                    f"из /tf собраны не обе руки ({sorted(models)}) — жду полное дерево")
            return False
        changed = any(models[s].names != self.models[s].names
                      or models[s].tip_link != self.models[s].tip_link
                      for s in ("left", "right")) \
            or not getattr(self, "_tf_models_ready", False)
        self.models = models
        for m in self.models.values():
            m.base_frame = self.base_frame
        if changed:
            for s, ex in self.exec.items():
                ex.model = self.models[s]
            self._tf_models_ready = True
            self._tf_warned = False
            self.get_logger().info(
                "дерево звеньев IK построено из /tf + /tf_static: " +
                ", ".join(f"{s}: {m.names[0]}..{m.names[-1]} -> {m.tip_link}"
                          for s, m in self.models.items()))
        return True

    def _demo_sides(self):
        side = str(self.get_parameter("demo_side").value)
        return ["left", "right"] if side == "both" else [side]

    def demo_tick(self):
        """Периодическая публикация демо-целей/дельт (внутренний таймер ноды)."""
        g = lambda n: float(self.get_parameter(n).value)
        now = self.get_clock().now().to_msg()
        if self.demo_goal_pubs:
            yaw = math.radians(g("demo_yaw_deg"))
            q = Rotation.from_euler("xyz", [yaw, 0.0, 0.0]).as_quat()
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
        self.joints.update(dict(zip(msg.name, msg.position)))

    def q_meas(self, model):
        return np.array([self.joints.get(n, 0.0) for n in model.names])

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
        if fid in ("", self.base_frame, "base_link"):
            return pose_to_T(goal.pose)
        tfm = self.tf_buffer.lookup_transform(self.base_frame, fid,
                                              rclpy.time.Time(),
                                              timeout=rclpy.duration.Duration(seconds=0.5))
        t = tfm.transform.translation
        r = tfm.transform.rotation
        T_bc = np.eye(4)
        T_bc[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T_bc[:3, 3] = [t.x, t.y, t.z]
        return T_bc @ pose_to_T(goal.pose)

    def seed_poses(self, model, q_now):
        """Текущая поза + несколько «типовых» поз — борьба с локальными минимумами DLS."""
        seeds = [q_now.copy()]
        home = clip_to_limits(np.zeros(7), model, margin=0.1)
        seeds.append(home)
        reach = np.array([-0.4, 0.3, 0.0, -0.9, 0.0, 0.2, 0.0])
        if model.side == "left":
            reach[1] = -reach[1]
        seeds.append(clip_to_limits(reach, model, margin=0.05))
        bent = np.array([-0.8, 0.5, 0.4, -1.6, 0.3, 0.4, 0.0])
        if model.side == "left":
            bent[1] = -bent[1]
        seeds.append(clip_to_limits(bent, model, margin=0.05))
        return seeds[: max(1, self.n_seeds)]

    # ------------------------------------------------------------ goals
    def set_goal(self, side, T_des):
        """Общий конвейер: IK -> траектория (с защитой стола) -> исполнение."""
        ex = self.exec[side]
        model = self.models[side]
        if self.guard is not None:
            T_des[:3, 3] = self.guard.clamp_target_above_table(T_des[:3, 3])
        ex.T_des = T_des
        q_now = self.q_meas(model)

        barrier = None
        if self.guard is not None:
            barrier = lambda q: self.guard.table_cost_gradient(model, q, k=30.0)

        # 1) IK: multi-seed DLS с барьером стола
        seeds = self.seed_poses(model, q_now)
        q_sol, ok, epos, erot = ik_multi_seed(
            model, T_des, seeds, max_iter=250, tol_pos=self.tol_pos * 0.5,
            tol_rot=self.tol_rot * 0.5, barrier=barrier)

        dt = 1.0 / self.rate_hz
        if ok and self.guard is not None:
            traj = joint_space_profile(q_now, q_sol, self.spd, self.a_max, dt)
            bad = any(self.guard.collides_with_table(model, qq) for qq in traj)
        else:
            bad = False

        if ok and not bad:
            ex.traj = joint_space_profile(q_now, q_sol, self.spd, self.a_max, dt)
            ex.idx, ex.state, ex.fine_cycles = 0, ArmExecutor.TRAJ, 0
            ex.last_T_des = T_des.copy()
            self.get_logger().info(
                f"[{side}] IK решена: {len(ex.traj)} точек (~{(len(ex.traj)-1)/self.rate_hz:.1f} c)")
            return True

        # 2) план Б: заход через waypoint строго над столом (две IK-фазы)
        if self.guard is not None:
            wp = T_des.copy()
            wp[:3, 3] = self.guard.clamp_target_above_table(wp[:3, 3] + np.array([0.0, 0.0, self.lift]))
            q_wp, ok1, _, _ = ik_multi_seed(model, wp, seeds, max_iter=200,
                                            barrier=barrier)
            q_fin, ok2, _, _ = ik_multi_seed(model, T_des, [q_wp] + seeds, max_iter=200,
                                             barrier=barrier)
            if ok1 and ok2:
                t1 = joint_space_profile(q_now, q_wp, self.spd, self.a_max, dt)
                t2 = joint_space_profile(q_wp, q_fin, self.spd, self.a_max, dt)
                ex.traj = np.vstack([t1[:-1], t2])
                ex.idx, ex.state, ex.fine_cycles = 0, ArmExecutor.TRAJ, 0
                ex.last_T_des = T_des.copy()
                self.get_logger().info(
                    f"[{side}] IK через waypoint над столом ({len(ex.traj)} точек)")
                return True

        ex.state, ex.traj = ArmExecutor.FAILED, None
        ex.last_err = (epos, erot)
        self.get_logger().error(
            f"[{side}] IK не сошлась (err pos={epos*100:.1f} см rot={erot:.2f} рад) — удержание")
        return False

    def on_goal(self, side, goal: PoseStamped):
        """Абсолютная шестимерная цель TCP (PoseStamped)."""
        try:
            T_des = self.to_base_T(goal)
        except Exception as e:  # TF ещё не доступен
            self.get_logger().error(f"[{side}] TF {self.base_frame}<-{goal.header.frame_id}: {e}")
            return
        self.set_goal(side, T_des)

    def tcp_pose_T(self, side):
        """Текущая измеренная поза TCP в base_frame (4x4) по /joint_states."""
        model = self.models[side]
        pos, quat_wxyz = fk_tcp(model, self.q_meas(model))
        return mat_from_posquat(pos, quat_wxyz)

    def on_delta(self, side, msg: TwistStamped):
        """Относительная цель: смещение TCP на 6-мерный вектор (TwistStamped).

        linear  = dxyz (м), angular = ось*угол (рад), оба в системе base_frame.
        Опорная поза (параметр delta_reference):
          'measured'  — текущая ИЗМЕРЕННАЯ поза TCP (FK от энкодеров);
                        повторная публикация той же дельты = «ещё раз на столько же»
                        относительно фактического положения кисти;
          'last_goal' — последняя поставленная абсолютная цель (идемпотентно).
        delta_accumulate=true — дельты суммируются от опорной точки первой дельты
        серии (джойстик-режим: каждая новая команда двигает дальше).
        """
        ex = self.exec[side]
        fid = msg.header.frame_id
        if fid not in ("", self.base_frame, "base_link"):
            self.get_logger().warn(
                f"[{side}] delta в рамке '{fid}' трактуется как {self.base_frame} (перенос не выполняется)")
        dpos = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        drot = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

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

        T_des = delta_to_T(T_ref, dpos, drot)
        self.get_logger().info(
            f"[{side}] delta-цель: dp=({dpos[0]:+.3f},{dpos[1]:+.3f},{dpos[2]:+.3f}) м, "
            f"dr=({drot[0]:+.3f},{drot[1]:+.3f},{drot[2]:+.3f}) рад "
            f"[ref={'last_goal' if use_last else 'measured'}]")
        self.set_goal(side, T_des)

    # ------------------------------------------------------------ service
    def solve_srv(self, req, res):
        side = req.side if req.side in ("left", "right") else "right"
        model = self.models[side]
        T_des = pose_to_T(req.goal.pose)
        q_init = np.zeros(7) if req.use_zero_seed else self.q_meas(model)
        q, ok, hist = ik_dls(model, T_des, q_init, max_iter=250)
        res.success = bool(ok)
        res.joint_names = model.names
        res.positions = [float(v) for v in q]
        res.motor_ids = [int(i) for i in model.ids]
        res.final_error = float(hist[-1]) if hist else 1e9
        return res

    # ------------------------------------------------------------ control loop
    def step(self):
        now = self.get_clock().now().to_msg()
        if self.tree_source == "tf":
            # периодическая пересборка дерева звеньев из /tf + /tf_static
            # (сначала — часто, пока дерево не получено; затем раз в tf_refresh)
            self.try_build_tree_from_tf()
        if self.tree_source == "tf" and not getattr(self, "_clock_checked", False):
            self._check_sim_clock()
        for side, model in self.models.items():
            ex = self.exec[side]
            q_meas = self.q_meas(model)

            # телеметрия: текущая поза TCP (FK от энкодеров)
            ps = T_to_posemsg(fk(model, q_meas)[0] @ _tmat(model.tcp_offset, [0, 0, 0]))
            ps.header.frame_id = self.base_frame
            ps.header.stamp = now
            self.pose_pubs[side].publish(ps)

            if ex.traj is not None and ex.idx < len(ex.traj):
                # фаза траектории: открытая петля по заранее просчитанному профилю
                if self.cmd_pub is not None:
                    self.cmd_pub.publish(self.make_cmd(model, ex.traj[ex.idx]))
                ex.idx += 1
            elif ex.state == ArmExecutor.TRAJ:
                ex.state = ArmExecutor.FINE  # профиль пройден -> дотягивание

            if ex.state == ArmExecutor.FINE:
                # фаза fine: замкнутый контур по /joint_states (недолёт ПД до 15 %)
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
                else:
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
