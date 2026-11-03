#!/usr/bin/env python3
"""object_detector_node.py — источник цели «внешний объект» для фокусировки головы.

nssr-sim, ROS 2 Humble. Python (rclpy): ML/CV-экосистема важнее скорости —
YOLO не нужен, хватает HSV-детектора + глубины (640x360, 10 Гц на CPU ок).

Математика (docs/FOCUS_TRACKING.md, раздел «Слежение за объектом»):
  1. Детекция: mask = inRange(HSV) по цвету объекта (цвет лотка/пробирки из /sim/task),
     largest connected component -> центр (u, v).
  2. Глубина: медиана d в окне 9x9 вокруг (u, v) на /cam_head/depth/image_raw (мм).
  3. Обратная проекция pinhole:  X=(u-cx)*d/fx, Y=(v-cy)*d/fy, Z=d.
  4. Публикация PoseStamped в /focus/target, frame_id=camera_head_link.

Интерфейсы:
  ВХОД  /cam_head/color/image_raw   sensor_msgs/Image (rgb8)
  ВХОД  /cam_head/depth/image_raw   sensor_msgs/Image (16UC1, мм, выровнена по color)
  ВХОД  /cam_head/color/camera_info sensor_msgs/CameraInfo
  ВХОД  /sim/task                   std_msgs/String (JSON, latched) — целевой цвет опционально
  ВЫХОД /focus/target               geometry_msgs/PoseStamped
  ПАРАМЕТР target_hsv               [h_lo,s_lo,v_lo, h_hi,s_hi,v_hi] — окно цвета цели
  ПАРАМЕТР min_area                 минимальная площадь blob в пикселях (по умолчанию 300)

Запуск:
  ros2 run focus_tracking object_detector_node --ros-args -p use_sim_time:=true \
    -p "target_hsv:=[0,100,100,10,255,255]"      # красный объект
"""

import json
import math

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")

        self.declare_parameter("target_hsv", [0.0, 100.0, 100.0, 10.0, 255.0, 255.0])
        self.declare_parameter("min_area", 300.0)
        self.declare_parameter("depth_window", 9)
        self.declare_parameter("max_range_m", 5.5)   # depth==0 => нет данных (>6 м)

        hsv = self.get_parameter("target_hsv").value
        self.h_lo, self.s_lo, self.v_lo = int(hsv[0]), int(hsv[1]), int(hsv[2])
        self.h_hi, self.s_hi, self.v_hi = int(hsv[3]), int(hsv[4]), int(hsv[5])
        self.min_area = float(self.get_parameter("min_area").value)
        self.dwin = int(self.get_parameter("depth_window").value)
        self.max_range = float(self.get_parameter("max_range_m").value)

        self.fx = self.fy = self.cx = self.cy = None

        self.pub = self.create_publisher(PoseStamped, "/focus/target", 10)

        self.info_sub = self.create_subscription(
            CameraInfo, "/cam_head/color/camera_info", self.on_info, 5)

        # синхронизация color+depth по времени (кадры выровнены в симе)
        self.color_sub = Subscriber(self, Image, "/cam_head/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/cam_head/depth/image_raw")
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.on_rgbd)

        # цель можно задать и цветом из задачи (/sim/task JSON) — берём color лотков при желании
        self.create_subscription(
            String, "/sim/task", self.on_task,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.get_logger().info(
            f"object_detector_node: HSV окно [{self.h_lo},{self.s_lo},{self.v_lo}].."
            f"[{self.h_hi},{self.s_hi},{self.v_hi}], min_area={self.min_area}")

    # ------------------------------------------------------------------ callbacks
    def on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def on_task(self, msg: String):
        try:
            task = json.loads(msg.data)
            self.get_logger().debug(f"/sim/task: {task.get('task')}")
        except (json.JSONDecodeError, AttributeError):
            pass

    @staticmethod
    def _to_np(img: Image) -> np.ndarray:
        n = np.frombuffer(img.data, dtype=np.uint16 if img.encoding == "16UC1" else np.uint8)
        return n.reshape(img.height, img.width, -1)

    def on_rgbd(self, color: Image, depth: Image):
        if self.fx is None:
            return

        bgr = self._to_np(color)[:, :, ::-1].copy()          # rgb8 -> bgr для OpenCV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        if self.h_lo <= self.h_hi:
            mask = cv2.inRange(hsv, (self.h_lo, self.s_lo, self.v_lo),
                                    (self.h_hi, self.s_hi, self.v_hi))
        else:  # красный: HSV-тон огибает 0 (170..180 + 0..10)
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (self.h_lo, self.s_lo, self.v_lo), (179, 255, 255)),
                cv2.inRange(hsv, (0, self.s_lo, self.v_lo), (self.h_hi, self.s_hi, self.v_hi)))

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        if n <= 1:
            return
        idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = float(stats[idx, cv2.CC_STAT_AREA])
        if area < self.min_area:
            return
        u, v = float(cents[idx][0]), float(cents[idx][1])

        # медиана глубины в окне вокруг центра (защита от краёв объектов/дыр)
        d_img = self._to_np(depth)[:, :, 0]
        w = self.dwin // 2
        y0, y1 = max(0, int(v) - w), min(d_img.shape[0], int(v) + w + 1)
        x0, x1 = max(0, int(u) - w), min(d_img.shape[1], int(u) + w + 1)
        vals = d_img[y0:y1, x0:x1]
        vals = vals[vals > 0]
        if vals.size == 0:
            return
        d = float(np.median(vals)) / 1000.0                  # мм -> м
        if not (0.15 < d < self.max_range):
            return

        # обратная проекция pinhole: пиксель + глубина -> 3D в camera_head_link
        x = (u - self.cx) * d / self.fx
        y = (v - self.cy) * d / self.fy

        ps = PoseStamped()
        ps.header.stamp = color.header.stamp
        ps.header.frame_id = "camera_head_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, d
        self.pub.publish(ps)

        yaw = math.atan2(x, d)
        pitch = math.atan2(-y, math.hypot(x, d))
        self.get_logger().debug(
            f"blob px({u:.0f},{v:.0f}) area={area:.0f} d={d:.2f}m -> "
            f"X={x:.2f} Y={y:.2f} yaw={yaw:.2f} pitch={pitch:.2f}")


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    try:
        rpy_spin = rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
