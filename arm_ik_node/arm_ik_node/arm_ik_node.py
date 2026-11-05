#!/usr/bin/env python3
"""
ROS 2 Node for Inverse Kinematics of TienKung 2.0 Pro Arms.
Reads URDF directly from a file path to avoid CLI argument issues.
"""

import os
import math
import xml.etree.ElementTree as ET
from collections import deque, defaultdict
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# Hackathon specific msgs
try:
    from bodyctrl_msgs.msg import CmdSetMotorPosition
    HAS_BODYCTRL = True
except ImportError:
    HAS_BODYCTRL = False

try:
    from scipy.optimize import least_squares
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# =============================================================================
# CONFIGURATION FOR TIENKUNG 2.0 PRO
# =============================================================================

LEFT_ARM_JOINT_MAP = {
    "shoulder_pitch_l": 11, "shoulder_roll_l": 12, "shoulder_yaw_l": 13,
    "elbow_pitch_l": 14, "elbow_yaw_l": 15, "wrist_pitch_l": 16, "wrist_roll_l": 17,
}

RIGHT_ARM_JOINT_MAP = {
    "shoulder_pitch_r": 21, "shoulder_roll_r": 22, "shoulder_yaw_r": 23,
    "elbow_pitch_r": 24, "elbow_yaw_r": 25, "wrist_pitch_r": 26, "wrist_roll_r": 27,
}

MAX_SPEED_RAD_S = 0.8
BASE_FRAME = "pelvis"

# =============================================================================
# MATH UTILITIES (Shortened for brevity, same as previous versions)
# =============================================================================

@dataclass
class Origin:
    xyz: np.ndarray
    rpy: np.ndarray

@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Origin
    axis: np.ndarray
    lower: float
    upper: float

@dataclass
class PathEdge:
    to_link: str
    joint: Joint
    joint_idx: Optional[int]
    forward: bool

def parse_vec3(value: Optional[str], default: Tuple[float, float, float]) -> np.ndarray:
    if value is None or value.strip() == "": return np.array(default, dtype=float)
    parts = value.split()
    if len(parts) != 3: raise ValueError(f"Bad vec3: {value}")
    return np.array([float(x) for x in parts], dtype=float)

def parse_float(value: Optional[str], default: float) -> float:
    if value is None: return default
    try: return float(value)
    except ValueError: return default

def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, -sy*cr+cy*sp*sr, sy*sr+cy*sp*cr],
        [sy*cp,  cy*cr+sy*sp*sr, -cy*sr+sy*sp*cr],
        [-sp,     cp*sr,         cp*cr]
    ], dtype=float)

def rot_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c = math.cos(angle); s = math.sin(angle); C = 1.0 - c
    return np.array([
        [c+x*x*C,     x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s,   c+y*y*C,   y*z*C-x*s],
        [z*x*C-y*s,   z*y*C+x*s, c+z*z*C]
    ], dtype=float)

def se3(p: np.ndarray, R: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float); T[:3,:3]=R; T[:3,3]=p; return T

def inv_se3(T: np.ndarray) -> np.ndarray:
    R=T[:3,:3]; p=T[:3,3]; Ti=np.eye(4,dtype=float)
    Ti[:3,:3]=R.T; Ti[:3,3]=-R.T@p; return Ti

def quat_to_matrix(q) -> np.ndarray:
    if hasattr(q, "x"): x,y,z,w = q.x,q.y,q.z,q.w
    else: x,y,z,w = map(float, q)
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n < 1e-12: return np.eye(3,dtype=float)
    x/=n; y/=n; z/=n; w/=n
    return np.array([
        [1.0-2.0*(y*y+z*z), 2.0*(x*y-z*w),     2.0*(x*z+y*w)],
        [2.0*(x*y+z*w),       1.0-2.0*(x*x+z*z), 2.0*(y*z-x*w)],
        [2.0*(x*z-y*w),       2.0*(y*z+x*w),       1.0-2.0*(x*x+y*y)]
    ], dtype=float)

def pose_to_matrix(pose) -> np.ndarray:
    R = quat_to_matrix(pose.orientation)
    p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    return se3(p, R)

def transform_stamped_to_matrix(ts) -> np.ndarray:
    p = np.array([ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z], dtype=float)
    R = quat_to_matrix(ts.transform.rotation)
    return se3(p, R)

def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    cos_angle = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_angle)
    if angle < 1e-8:
        return 0.5 * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    if math.pi - angle < 1e-6:
        if R[0,0]>R[1,1] and R[0,0]>R[2,2]:
            x=math.sqrt(max(0,(R[0,0]+1)/2)); y=(R[0,1]+R[1,0])/(4*x) if x>1e-12 else 0; z=(R[0,2]+R[2,0])/(4*x) if x>1e-12 else 0
        elif R[1,1]>R[2,2]:
            y=math.sqrt(max(0,(R[1,1]+1)/2)); x=(R[0,1]+R[1,0])/(4*y) if y>1e-12 else 0; z=(R[1,2]+R[2,1])/(4*y) if y>1e-12 else 0
        else:
            z=math.sqrt(max(0,(R[2,2]+1)/2)); x=(R[0,2]+R[2,0])/(4*z) if z>1e-12 else 0; y=(R[1,2]+R[2,1])/(4*z) if z>1e-12 else 0
        return np.array([x,y,z])*math.pi
    s = math.sin(angle)
    w = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return (angle / (2.0 * s)) * w

# =============================================================================
# URDF PARSER & KINEMATIC TREE
# =============================================================================

def parse_urdf(urdf_xml: str) -> List[Joint]:
    root = ET.fromstring(urdf_xml)
    joints: List[Joint] = []
    for joint_el in root.findall("joint"):
        name = joint_el.get("name"); jtype = joint_el.get("type", "fixed")
        parent_el = joint_el.find("parent"); child_el = joint_el.find("child")
        if parent_el is None or child_el is None: continue
        parent = parent_el.get("link"); child = child_el.get("link")
        if not all([name, parent, child]): continue

        origin_el = joint_el.find("origin")
        if origin_el is None: xyz = np.zeros(3); rpy = np.zeros(3)
        else:
            xyz = parse_vec3(origin_el.get("xyz"), (0,0,0))
            rpy = parse_vec3(origin_el.get("rpy"), (0,0,0))

        axis_el = joint_el.find("axis")
        if axis_el is None: axis = np.array([1.0, 0.0, 0.0])
        else: axis = parse_vec3(axis_el.get("xyz"), (1.0, 0.0, 0.0))
        
        norm = np.linalg.norm(axis)
        axis = axis/norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0])

        lower, upper = float('nan'), float('nan')
        limit_el = joint_el.find("limit")
        if limit_el is not None:
            lower = parse_float(limit_el.get("lower"), float('nan'))
            upper = parse_float(limit_el.get("upper"), float('nan'))

        joints.append(Joint(name, jtype, parent, child, Origin(xyz, rpy), axis, lower, upper))
    return joints

class KinematicTree:
    def __init__(self, joints: List[Joint]):
        self.joints = joints
        self.adj = defaultdict(list)
        for j in joints:
            self.adj[j.parent].append((j.child, j, True))
            self.adj[j.child].append((j.parent, j, False))

    def find_path(self, start: str, goal: str) -> Tuple[List[Joint], List[PathEdge]]:
        if start == goal: return [], []
        queue = deque([(start, [])]); visited = {start}
        while queue:
            link, raw_edges = queue.popleft()
            if link == goal: return self._make_path(raw_edges)
            for next_link, joint, forward in self.adj[link]:
                if next_link not in visited:
                    visited.add(next_link)
                    queue.append((next_link, raw_edges + [(next_link, joint, forward)]))
        raise ValueError(f"Kinematic path not found: {start} -> {goal}")

    @staticmethod
    def _make_path(raw_edges) -> Tuple[List[Joint], List[PathEdge]]:
        movable_joints: List[Joint] = []; joint_name_to_idx: Dict[str, int] = {}; path_edges: List[PathEdge] = []
        for next_link, joint, forward in raw_edges:
            if joint.type == "fixed": idx = None
            else:
                if joint.name not in joint_name_to_idx:
                    joint_name_to_idx[joint.name] = len(movable_joints)
                    movable_joints.append(joint)
                idx = joint_name_to_idx[joint.name]
            path_edges.append(PathEdge(to_link=next_link, joint=joint, joint_idx=idx, forward=forward))
        return movable_joints, path_edges


# =============================================================================
# ROS 2 NODE
# =============================================================================

class TienKungIKNode(Node):
    def __init__(self):
        super().__init__("tienkung_ik_node")

        # --- Parameters ---
        # NEW PARAMETER: Path to URDF file
        self.declare_parameter("urdf_path", "") 
        self.declare_parameter("side", "left") # 'left' or 'right'
        self.declare_parameter("target_pose_topic", "/left_arm/target_pose")
        self.declare_parameter("output_cmd_topic", "/arm/cmd_pos")
        self.declare_parameter("offset_topic", "/left_arm/ee_offset")
        self.declare_parameter("orientation_weight", 1.0)
        self.declare_parameter("ik_tolerance", 1e-3)
        self.declare_parameter("publish_offset", True)
        self.declare_parameter("offset_rate_hz", 10.0)
        self.declare_parameter("max_speed_rad_s", MAX_SPEED_RAD_S)

        self.urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value
        self.side = self.get_parameter("side").get_parameter_value().string_value.lower()
        
        if self.side not in ("left", "right"):
            self.get_logger().error("Side must be 'left' or 'right'. Defaulting to left.")
            self.side = "left"

        self.target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        self.output_cmd_topic = self.get_parameter("output_cmd_topic").get_parameter_value().string_value
        self.offset_topic = self.get_parameter("offset_topic").get_parameter_value().string_value
        
        self.orientation_weight = float(self.get_parameter("orientation_weight").value)
        self.ik_tolerance = float(self.get_parameter("ik_tolerance").value)
        self.max_speed = float(self.get_parameter("max_speed_rad_s").value)
        
        self.publish_offset = bool(self.get_parameter("publish_offset").value)
        self.offset_rate_hz = float(self.get_parameter("offset_rate_hz").value)

        # Select correct EE frame and Joint Map based on side
        if self.side == "left":
            self.ee_frame = "wrist_roll_l_link"
            self.joint_map = LEFT_ARM_JOINT_MAP
        else:
            self.ee_frame = "wrist_roll_r_link"
            self.joint_map = RIGHT_ARM_JOINT_MAP
            
        self.base_frame = BASE_FRAME

        # --- State ---
        self.loaded = False
        self.tree: Optional[KinematicTree] = None
        self.movable_joints: List[Joint] = []
        self.path_edges: List[PathEdge] = []
        self.lower_bounds = np.array([])
        self.upper_bounds = np.array([])
        self.static_T_map: Dict[str, np.ndarray] = {} 
        
        self.joint_positions: Dict[str, float] = {}
        self.last_target_T: Optional[np.ndarray] = None

        # --- Interfaces ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, self.target_pose_topic, self.target_pose_cb, 10)
        
        # Publishers
        if HAS_BODYCTRL:
            self.cmd_pub = self.create_publisher(CmdSetMotorPosition, self.output_cmd_topic, 10)
        else:
            self.get_logger().warn("bodyctrl_msgs not found. Using JointState fallback.")
            self.cmd_pub = self.create_publisher(JointState, self.output_cmd_topic, 10)

        self.offset_pub = self.create_publisher(TwistStamped, self.offset_topic, 10)

        # Timers
        self.create_timer(1.0, self.try_load_model)
        if self.publish_offset and self.offset_rate_hz > 0:
            period = 1.0 / self.offset_rate_hz
            self.create_timer(period, self.publish_offset_cb)

        if not HAS_SCIPY:
            self.get_logger().error("SciPy not installed!")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = float(pos)

    # -------------------------------------------------------------------------
    # Model Loading (FILE READ LOGIC HERE)
    # -------------------------------------------------------------------------

    def try_load_model(self):
        if self.loaded: return
        
        urdf_str = ""
        
        # 1. Try reading from parameter robot_description (standard ROS way)
        param_desc = self.get_parameter("robot_description").get_parameter_value().string_value
        if param_desc:
            urdf_str = param_desc
            self.get_logger().info("Loaded URDF from parameter 'robot_description'.")
        else:
            # 2. Fallback: Read from file specified by 'urdf_path'
            if not self.urdf_path:
                self.get_logger().error("Both 'robot_description' param and 'urdf_path' are empty. Cannot load model.")
                return
            
            if not os.path.exists(self.urdf_path):
                self.get_logger().error(f"URDF file not found at: {self.urdf_path}")
                return

            try:
                with open(self.urdf_path, 'r', encoding='utf-8') as f:
                    urdf_str = f.read()
                self.get_logger().info(f"Successfully loaded URDF from file: {self.urdf_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to read URDF file: {e}")
                return

        if not urdf_str:
            self.get_logger().error("URDF string is empty after loading.")
            return

        try:
            joints = parse_urdf(urdf_str)
            self.tree = KinematicTree(joints)
            self.movable_joints, self.path_edges = self.tree.find_path(self.base_frame, self.ee_frame)
            self.lower_bounds, self.upper_bounds = self.make_bounds(self.movable_joints)
            
            # Precompute static transforms
            self.static_T_map.clear()
            for edge in self.path_edges:
                if edge.joint_idx is None:
                    key = f"{edge.joint.parent}_{edge.joint.child}"
                    T_orig = se3(edge.joint.origin.xyz, rpy_to_matrix(edge.joint.origin.rpy))
                    self.static_T_map[key] = T_orig if edge.forward else inv_se3(T_orig)
            
            # Verify mappings
            missing_ids = [j.name for j in self.movable_joints if j.name not in self.joint_map]
            if missing_ids:
                self.get_logger().warn(f"Some joints not mapped to motor IDs: {missing_ids}")

            self.get_logger().info(f"IK Chain Loaded ({self.side}): {len(self.movable_joints)} DOFs.")
            self.loaded = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse/build kinematic tree: {e}")

    def make_bounds(self, joints: List[Joint]):
        lower, upper = [], []
        for j in joints:
            lo, hi = j.lower, j.upper
            if j.type in ("revolute", "continuous"):
                if not math.isfinite(lo): lo = -math.pi
                if not math.isfinite(hi): hi = math.pi
            elif j.type == "prismatic":
                if not math.isfinite(lo): lo = -1.0
                if not math.isfinite(hi): hi = 1.0
            else:
                lo, hi = 0.0, 0.0
                
            if lo > hi: lo, hi = hi, lo
            if hi - lo < 1e-9:
                lo -= 1e-6; hi += 1e-6
                
            lower.append(lo); upper.append(hi)
        return np.array(lower), np.array(upper)

    # -------------------------------------------------------------------------
    # Forward Kinematics
    # -------------------------------------------------------------------------

    def fk(self, q: np.ndarray) -> np.ndarray:
        T = np.eye(4, dtype=float)
        for i, edge in enumerate(self.path_edges):
            if edge.joint_idx is None:
                key = f"{edge.joint.parent}_{edge.joint.child}"
                T_joint = self.static_T_map[key]
            else:
                q_val = float(q[edge.joint_idx])
                j = edge.joint
                T_origin = se3(j.origin.xyz, rpy_to_matrix(j.origin.rpy))
                
                if j.type == "fixed":
                    T_joint = T_origin
                elif j.type in ("revolute", "continuous"):
                    R = rot_axis(j.axis, q_val)
                    T_joint = T_origin @ se3(np.zeros(3), R)
                elif j.type == "prismatic":
                    p = j.axis * q_val
                    T_joint = T_origin @ se3(p, np.eye(3))
                else:
                    T_joint = T_origin
                    
                if not edge.forward:
                    T_joint = inv_se3(T_joint)
            
            T = T @ T_joint
        return T

    # -------------------------------------------------------------------------
    # Inverse Kinematics Solver
    # -------------------------------------------------------------------------

    def initial_guess(self) -> np.ndarray:
        q0 = []
        for i, j in enumerate(self.movable_joints):
            val = self.joint_positions.get(j.name)
            if val is None or not math.isfinite(val):
                val = 0.5 * (self.lower_bounds[i] + self.upper_bounds[i])
            val = float(np.clip(val, self.lower_bounds[i], self.upper_bounds[i]))
            q0.append(val)
        return np.array(q0)

    def solve_ik(self, target_T: np.ndarray, q0: np.ndarray):
        if len(self.movable_joints) == 0:
            return np.array([]), 0.0

        def residual(x: np.ndarray) -> np.ndarray:
            T_curr = self.fk(x)
            pos_err = T_curr[:3, 3] - target_T[:3, 3]
            rot_err = matrix_to_rotvec(target_T[:3, :3].T @ T_curr[:3, :3])
            return np.concatenate([pos_err, self.orientation_weight * rot_err])

        result = least_squares(
            residual, q0,
            bounds=(self.lower_bounds, self.upper_bounds),
            method='trf',
            xtol=1e-9, ftol=1e-9, max_nfev=200
        )
        err = float(np.linalg.norm(result.fun))
        return result.x, err

    # -------------------------------------------------------------------------
    # Target Processing & Output Generation
    # -------------------------------------------------------------------------

    def transform_pose_to_base(self, msg: PoseStamped) -> np.ndarray:
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.base_frame
        out.pose = msg.pose

        if out.header.frame_id != self.base_frame:
            ts = self.tf_buffer.lookup_transform(
                self.base_frame, out.header.frame_id, Time(), timeout=Duration(seconds=0.1)
            )
            do_transform_pose(out, ts)
        
        return pose_to_matrix(out.pose)

    def get_current_ee_pose(self) -> Optional[np.ndarray]:
        if not self.loaded: return None
        try:
            ts = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, Time(), timeout=Duration(seconds=0.05)
            )
            return transform_stamped_to_matrix(ts)
        except Exception:
            pass
        q = self.current_joint_vector()
        try:
            return self.fk(q)
        except Exception:
            return None

    def current_joint_vector(self) -> np.ndarray:
        if not self.loaded: return np.array([])
        q = []
        for i, j in enumerate(self.movable_joints):
            val = self.joint_positions.get(j.name)
            if val is None or not math.isfinite(val):
                val = 0.5 * (self.lower_bounds[i] + self.upper_bounds[i])
            val = float(np.clip(val, self.lower_bounds[i], self.upper_bounds[i]))
            q.append(val)
        return np.array(q)

    def compute_offset_twist(self, T_current: np.ndarray, T_target: np.ndarray) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        R_c = T_current[:3, :3]; p_c = T_current[:3, 3]
        R_t = T_target[:3, :3]; p_t = T_target[:3, 3]

        p_err = p_t - p_c
        R_err = R_t.T @ R_c 
        rot_err = matrix_to_rotvec(R_err)

        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(p_err[0]); msg.twist.linear.y = float(p_err[1]); msg.twist.linear.z = float(p_err[2])
        msg.twist.angular.x = float(rot_err[0]); msg.twist.angular.y = float(rot_err[1]); msg.twist.angular.z = float(rot_err[2])
        
        return msg

    def target_pose_cb(self, msg: PoseStamped):
        if not self.loaded:
            self.get_logger().warn_throttle(5.0, "Model not loaded yet.")
            return
        if not HAS_SCIPY:
            return

        try:
            target_T = self.transform_pose_to_base(msg)
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f"Transform failed: {e}")
            return

        self.last_target_T = target_T.copy()

        if self.publish_offset:
            T_curr = self.get_current_ee_pose()
            if T_curr is not None:
                self.offset_pub.publish(self.compute_offset_twist(T_curr, target_T))

        q0 = self.initial_guess()
        try:
            q_sol, err = self.solve_ik(target_T, q0)
        except Exception as e:
            self.get_logger().error(f"IK Failed: {e}")
            return

        if err > self.ik_tolerance:
            self.get_logger().warn(f"IK Error High: {err:.6f}")

        self.publish_commands(q_sol)
        self.get_logger().debug(f"IK Solved. Err: {err:.6f}")

    def publish_commands(self, q_sol: np.ndarray):
        if not HAS_BODYCTRL:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [j.name for j in self.movable_joints]
            js.position = [float(v) for v in q_sol]
            self.cmd_pub.publish(js)
            return

        cmd_msg = CmdSetMotorPosition()
        cmds_list = []

        for i, joint in enumerate(self.movable_joints):
            motor_id = self.joint_map.get(joint.name)
            if motor_id is None: continue
            
            target_pos = float(q_sol[i])
            current_pos = self.joint_positions.get(joint.name, target_pos)
            delta = abs(target_pos - current_pos)
            speed = min(MAX_SPEED_RAD_S, max(0.1, delta * 10.0)) 
            
            cmd_item = {
                "name": motor_id,
                "pos": target_pos,
                "spd": speed,
                "cur": 0.0
            }
            cmds_list.append(cmd_item)

        if cmds_list:
            cmd_msg.cmds = cmds_list
            self.cmd_pub.publish(cmd_msg)

    def publish_offset_cb(self):
        if not self.publish_offset or self.last_target_T is None:
            return
        T_curr = self.get_current_ee_pose()
        if T_curr is None: return
        msg = self.compute_offset_twist(T_curr, self.last_target_T)
        self.offset_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TienKungIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()