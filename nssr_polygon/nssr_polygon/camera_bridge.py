#!/usr/bin/env python3
# Publishes the simulator's head and torso RGB-D images and camera info to ROS 2.
# Author: Nikolay Semenov <nssr.nikolay@gmail.com>
import array
import json
import os
import threading

import rclpy
import yaml
import zmq
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from builtin_interfaces.msg import Time
from std_msgs.msg import Header


def _stamp(t: float) -> Time:
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


class CameraBridge(Node):
    def __init__(self, cfg: dict):
        super().__init__("hack_camera_bridge")
        self.cfg = cfg
        qos = QoSProfile(depth=int(cfg.get("qos_depth", 2)),
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pubs = {}
        for name, c in cfg["cameras"].items():
            self.pubs[name] = {
                "color": self.create_publisher(Image, c["color_topic"], qos),
                "color_info": self.create_publisher(CameraInfo, c["color_info_topic"], qos),
                "depth": self.create_publisher(Image, c["depth_topic"], qos),
                "depth_info": self.create_publisher(CameraInfo, c["depth_info_topic"], qos),
                "frame_id": c["frame_id"],
            }
        self.counts = {n: 0 for n in self.pubs}
        ctx = zmq.Context.instance()
        self.sock = ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.RCVHWM, 4)
        self.sock.connect(f"tcp://{cfg['zmq']['host']}:{cfg['zmq']['port']}")
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"camera bridge up: {list(self.pubs)}")

    def _report(self):
        self.get_logger().info("frames/5s: " + ", ".join(f"{k}={v}" for k, v in self.counts.items()))
        self.counts = {n: 0 for n in self.pubs}

    def _info(self, meta, header):
        ci = CameraInfo()
        ci.header = header
        ci.width, ci.height = meta["width"], meta["height"]
        K = [float(x) for x in meta["K"]]
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = K
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [K[0], K[1], K[2], 0.0, K[3], K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _image(self, header, w, h, enc, step, buf):
        m = Image()
        m.header = header
        m.width, m.height, m.encoding, m.is_bigendian, m.step = w, h, enc, 0, step
        m.data = array.array("B", buf)
        return m

    def _loop(self):
        while self._run and rclpy.ok():
            try:
                parts = self.sock.recv_multipart()
            except zmq.ZMQError as e:
                self.get_logger().error(f"zmq: {e}")
                continue
            if len(parts) < 2:
                continue
            try:
                meta = json.loads(parts[0])
            except ValueError:
                continue
            p = self.pubs.get(meta.get("name"))
            if p is None:
                continue
            w, h = meta["width"], meta["height"]
            header = Header(stamp=_stamp(meta["stamp"]), frame_id=p["frame_id"])
            info = self._info(meta, header)
            p["color"].publish(self._image(header, w, h, "rgb8", w * 3, parts[1]))
            p["color_info"].publish(info)
            if len(parts) > 2 and len(parts[2]) == w * h * 2:
                p["depth"].publish(self._image(header, w, h, "16UC1", w * 2, parts[2]))
                p["depth_info"].publish(info)
            self.counts[meta["name"]] += 1


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.environ.get("HACK_CAM_CFG", os.path.join(here, "..", "config", "cameras.yaml"))
    if not os.path.isfile(path):
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("nssr_polygon"), "config", "cameras.yaml")
        except Exception:
            pass
    with open(path) as f:
        cfg = yaml.safe_load(f)
    rclpy.init()
    node = CameraBridge(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._run = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
