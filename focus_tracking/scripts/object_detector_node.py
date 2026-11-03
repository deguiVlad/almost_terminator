#!/usr/bin/env python3
"""object_detector_node.py — источник цели «внешний объект» для фокусировки головы.

nssr-sim, ROS 2 Humble. Python (rclpy): ML/CV-экосистема важнее скорости —
YOLO не нужен, хватает HSV-детектора + глубины (640x360, 10 Гц на CPU ок).

Математика (docs/FOCUS_TRACKING.md, раздел «Слежение за объектом»):
  1. Детекция: все blob-ы, попавшие в любое из именованных HSV-окон (цветов),
     largest connected component по каждому окну -> центр (u, v).
  2. Глубина: медиана d в окне 9x9 вокруг (u, v) на /cam_head/depth/image_raw (мм).
  3. Обратная проекция pinhole:  X=(u-cx)*d/fx, Y=(v-cy)*d/fy, Z=d.
  4. Публикация PoseStamped в /focus/target, frame_id=camera_head_link.

Ручной интерфейс (сервис /focus/detect, focus_tracking/srv/DetectObjects):
  - без select_*  -> возвращает список всех обнаруженных объектов (id, имя/цвет,
    площадь, 3D-позиция); цель не меняется;
  - select_name='red' | select_id=N -> публикует выбранную цель в /focus/target
    (режим авто-выбора самого большого blob при этом отключается).

Интерфейсы:
  ВХОД  /cam_head/color/image_raw   sensor_msgs/Image (rgb8)
  ВХОД  /cam_head/depth/image_raw   sensor_msgs/Image (16UC1, мм, выровнена по color)
  ВХОД  /cam_head/color/camera_info sensor_msgs/CameraInfo
  ВЫХОД /focus/target               geometry_msgs/PoseStamped
  СЕРВИС /focus/detect              focus_tracking/srv/DetectObjects
  ПАРАМЕТР colors                   dict имя->HSV окно {name:[h_lo,s_lo,v_lo,h_hi,s_hi,v_hi]}
  ПАРАМЕТР min_area                 минимальная площадь blob в пикселях (по умолчанию 300)
  ПАРАМЕТР auto_select              true — публиковать самый большой blob сам (без сервиса)

Запуск:
  ros2 run focus_tracking object_detector_node --ros-args -p use_sim_time:=true \
    -p auto_select:=false
"""

import math

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped

try:
    from focus_tracking.srv import DetectObjects
except ImportError:      # пакет ещё не собран — srv недоступен, но нодa работает в auto режиме
    DetectObjects = None


# Цвета по умолчанию: тон красного огибает 0 (170..179 + 0..10)
DEFAULT_COLORS = {
    "red":    [170, 100, 80, 10, 255, 255],
    "green":  [40, 80, 60, 85, 255, 255],
    "blue":   [95, 80, 60, 130, 255, 255],
    "yellow": [20, 80, 80, 38, 255, 255],
}


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")

        self.declare_parameter("colors", [])          # плоский список: name,h_lo,...,v_hi x N
        self.declare_parameter("min_area", 300.0)
        self.declare_parameter("depth_window", 9)
        self.declare_parameter("max_range_m", 5.5)   # depth==0 => нет данных (>6 м)
        self.declare_parameter("auto_select", True)

        self.min_area = float(self.get_parameter("min_area").value)
        self.dwin = int(self.get_parameter("depth_window").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.auto_select = bool(self.get_parameter("auto_select").value)

        flat = self.get_parameter("colors").value
        if flat:
            self.colors = {}
            for i in range(0, len(flat), 7):
                name = str(flat[i])
                self.colors[name] = [float(x) for x in flat[i + 1:i + 7]]
        else:
            self.colors = dict(DEFAULT_COLORS)

        self.fx = self.fy = self.cx = self.cy = None
        self.last_frame = None       # (color_msg, depth_msg) — последний синхронизированный кадр
        self.selected = None         # (kind, value) выбор оператора: ('name', s) | ('id', n)

        self.pub = self.create_publisher(PoseStamped, "/focus/target", 10)

        self.info_sub = self.create_subscription(
            CameraInfo, "/cam_head/color/camera_info", self.on_info, 5)

        # синхронизация color+depth по времени (кадры выровнены в симе)
        self.color_sub = Subscriber(self, Image, "/cam_head/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/cam_head/depth/image_raw")
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.on_rgbd)

        if DetectObjects is not None:
            self.srv = self.create_service(
                DetectObjects, "/focus/detect", self.on_detect)
        else:
            self.srv = None
            self.get_logger().warn(
                "srv/DetectObjects недоступен (соберите пакет), сервис /focus/detect выключен")

        names = ", ".join(self.colors)
        self.get_logger().info(
            f"object_detector_node: цвета [{names}], min_area={self.min_area}, "
            f"auto_select={self.auto_select}")

    # ------------------------------------------------------------------ callbacks
    def on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    @staticmethod
    def _to_np(img: Image) -> np.ndarray:
        n = np.frombuffer(img.data, dtype=np.uint16 if img.encoding == "16UC1" else np.uint8)
        return n.reshape(img.height, img.width, -1)

    def on_rgbd(self, color: Image, depth: Image):
        self.last_frame = (color, depth)
        if self.auto_select and self.selected is None and self.fx is not None:
            objs = self.detect(color, depth)
            if objs:
                self.publish_target(objs[0]["pose"])

    # -------------------------------------------------------------- детекция
    def detect(self, color: Image, depth: Image):
        """Список объектов [{name,id,area,u,v,depth,d_x,y,z,pose}] по всем цветам."""
        bgr = self._to_np(color)[:, :, ::-1].copy()          # rgb8 -> bgr для OpenCV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        d_img = self._to_np(depth)[:, :, 0]
        w = self.dwin // 2

        out, next_id = [], 1
        for name, win in self.colors.items():
            h_lo, s_lo, v_lo, h_hi, s_hi, v_hi = win
            if h_lo <= h_hi:
                mask = cv2.inRange(hsv, (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))
            else:  # красный: HSV-тон огибает 0 (170..179 + 0..10)
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, (h_lo, s_lo, v_lo), (179, 255, 255)),
                    cv2.inRange(hsv, (0, s_lo, v_lo), (h_hi, s_hi, v_hi)))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
            if n <= 1:
                continue
            idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            area = float(stats[idx, cv2.CC_STAT_AREA])
            if area < self.min_area:
                continue
            u, v = float(cents[idx][0]), float(cents[idx][1])

            # медиана глубины в окне вокруг центра (защита от краёв объектов/дыр)
            y0, y1 = max(0, int(v) - w), min(d_img.shape[0], int(v) + w + 1)
            x0, x1 = max(0, int(u) - w), min(d_img.shape[1], int(u) + w + 1)
            vals = d_img[y0:y1, x0:x1]
            vals = vals[vals > 0]
            if vals.size == 0:
                continue
            d = float(np.median(vals)) / 1000.0                  # мм -> м
            if not (0.15 < d < self.max_range):
                continue

            # обратная проекция pinhole: пиксель + глубина -> 3D в camera_head_link
            x = (u - self.cx) * d / self.fx
            y = (v - self.cy) * d / self.fy

            ps = PoseStamped()
            ps.header.stamp = color.header.stamp
            ps.header.frame_id = "camera_head_link"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, d

            yaw = math.atan2(x, d)
            pitch = math.atan2(-y, math.hypot(x, d))
            out.append(dict(name=name, id=next_id, area=area, u=u, v=v,
                            depth=d, x=x, y=y, z=d, pose=ps, yaw=yaw, pitch=pitch))
            next_id += 1

        out.sort(key=lambda o: -o["area"])          # id присваиваем после сортировки
        for i, o in enumerate(out):
            o["id"] = i + 1
        return out

    # -------------------------------------------------------------- сервис
    def on_detect(self, request, response):
        if self.last_frame is None or self.fx is None:
            response.message = "кадр ещё не получен (ждём /cam_head/*)"
            response.selected_ok = False
            return response

        objs = self.detect(*self.last_frame)
        response.names = [o["name"] for o in objs]
        response.ids = [o["id"] for o in objs]
        response.areas_px = [o["area"] for o in objs]
        response.targets = [o["pose"] for o in objs]

        sel = None
        if request.select_name:
            sel = next((o for o in objs if o["name"] == request.select_name), None)
            if sel is None:
                response.message = (f"цвет '{request.select_name}' не обнаружен; "
                                    f"доступно: {response.names}")
        elif request.select_id:
            sel = next((o for o in objs if o["id"] == request.select_id), None)
            if sel is None:
                response.message = f"id {request.select_id} не найден (1..{len(objs)})"
        else:
            response.message = (f"обнаружено {len(objs)} шт: " +
                                ", ".join(f"{o['id']}:{o['name']}({o['area']:.0f}px,"
                                          f"d={o['depth']:.2f}m)" for o in objs))

        if sel is not None:
            self.selected = None if not self.auto_select else ("set", sel["id"])
            self.publish_target(sel["pose"])
            response.selected_ok = True
            self.get_logger().info(
                f"выбрана цель {sel['id']}:{sel['name']} "
                f"yaw={sel['yaw']:.2f} pitch={sel['pitch']:.2f}")
            if not response.message:
                response.message = f"фокус на {sel['id']}:{sel['name']}"
        return response

    def publish_target(self, ps: PoseStamped):
        self.pub.publish(ps)


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
