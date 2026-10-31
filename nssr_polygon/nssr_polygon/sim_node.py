#!/usr/bin/env python3
# ROS 2 node of the simulator: task description, object names, layouts on request and the self-check.
# Author: Nikolay Semenov <nssr.nikolay@gmail.com>
import json, os, queue, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Int32
import zmq


class HackSimNode(Node):
    def __init__(self):
        super().__init__("hack_sim_node")
        ctx = zmq.Context()
        self.cmd = ctx.socket(zmq.PUB)
        self.cmd.bind(f"tcp://*:{os.environ.get('HACK_ZMQ_PORT', '5558')}")
        self._out = queue.Queue()
        threading.Thread(target=self._sender, daemon=True).start()
        self.st = ctx.socket(zmq.SUB)
        self.st.setsockopt(zmq.CONFLATE, 1)
        self.st.setsockopt_string(zmq.SUBSCRIBE, "")
        self.st.connect("tcp://127.0.0.1:5556")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_names = self.create_publisher(String, "/sim/objects/names", latched)
        self.pub_task = self.create_publisher(String, "/sim/task", latched)
        self.pub_result = self.create_publisher(String, "/sim/task/result", latched)
        self.create_subscription(Int32, "/sim/task/check", self.on_check, 10)
        self._task = self._result = None
        self.create_subscription(String, "/sim/objects/set", self.on_set, 10)
        self.create_subscription(Int32, "/sim/objects/randomize", self.on_rand, 10)
        self._names = None
        self.shaper = None
        if os.environ.get("HACK_SHAPER", "1") == "1":
            try:
                try:
                    from .robot_commands import Shaper
                except ImportError:
                    from robot_commands import Shaper
                self.shaper = Shaper(self, self.send)
            except Exception as e:
                self.get_logger().error(f"shaper off: {e}")
        threading.Thread(target=self._poll, daemon=True).start()
        self.get_logger().info("hack_sim_node: /sim/objects/{set,randomize,names}, /sim/task{,/check,/result}")

    def on_set(self, msg):
        try:
            d = json.loads(msg.data)
            assert isinstance(d, dict)
        except Exception as e:
            self.get_logger().error(f"/sim/objects/set: bad JSON ({e}); expected {{name: [x, y, yaw]}}")
            return
        self.send({"hack_objects_set": d})
        self.get_logger().info(f"/sim/objects/set -> sim: {list(d)}")

    def on_check(self, msg):
        self.send({"hack_task_check": int(msg.data)})
        self.get_logger().info("/sim/task/check -> sim")

    def on_rand(self, msg):
        self.send({"hack_objects_randomize": int(msg.data)})
        self.get_logger().info(f"/sim/objects/randomize -> sim: seed {msg.data}")

    def send(self, d):
        self._out.put(d)

    def _sender(self):
        while True:
            d = self._out.get()
            try:
                self.cmd.send_json(d)
            except Exception as e:
                self.get_logger().warn(f"zmq send: {e}")

    def _poll(self):
        while rclpy.ok():
            try:
                if self.st.poll(500):
                    st = self.st.recv_json()
                    if self.shaper is not None:
                        self.shaper.on_status(st)
                    names, task, res = st.get("hack_objects"), st.get("hack_task"), st.get("hack_task_result")
                    if names is not None and names != self._names:
                        self._names = names
                        self.pub_names.publish(String(data=json.dumps(names)))
                    if task is not None and task != self._task:
                        self._task = task
                        self.pub_task.publish(String(data=json.dumps(task, ensure_ascii=False)))
                    if res is not None and res != self._result:
                        self._result = res
                        self.pub_result.publish(String(data=json.dumps(res, ensure_ascii=False)))
            except Exception as e:
                self.get_logger().warn(f"status poll: {e}")


def main():
    rclpy.init()
    n = HackSimNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
