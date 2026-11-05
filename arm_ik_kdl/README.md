# arm_ik_kdl

ROS 2 Humble C++ inverse-kinematics server for the TienKung 2.0 Pro arms.
This package is intentionally separate from the existing Python `arm_ik`
package: it publishes joint-space solutions for integration/testing and does
**not** send actuator commands.

## Architecture

- `HumanoidIkNode`: ROS 2 orchestration, URDF loading, joint-state ingestion,
  target state, timer and publishers.
- One independent `Arm` context per side: KDL chain, joint limits, FK solver,
  velocity IK solver and position IK solver.
- URDF is the single source of truth. Chains are extracted from
  `waist_yaw_link` to `left_tcp_link` and `right_tcp_link`; the TCP fixed
  segment is included by KDL.
- IK uses current measured joint positions as the seed. Failed solutions are
  not published.
- Absolute targets and incremental transforms update a held target. A 100 Hz
  timer solves against the latest state; the target remains active until
  replaced.

## Interfaces

| Side | Absolute target | Relative increment | Joint solution |
|---|---|---|---|
| left | `/left_arm/target_pose` (`PoseStamped`) | `/left_arm/target_delta` (`TransformStamped`) | `/left_arm/ik_solution` (`JointState`) |
| right | `/right_arm/target_pose` (`PoseStamped`) | `/right_arm/target_delta` (`TransformStamped`) | `/right_arm/ik_solution` (`JointState`) |

Absolute target position and quaternion are interpreted in `waist_yaw_link`.
Delta is a rigid transform in the **local TCP frame** and is accumulated as
`T_target_new = T_target_old * T_delta`. It is not a velocity command.
The `header.frame_id` is currently informational; no TF conversion is done.

The node subscribes to `/joint_states`. Each side's target is initialized from
measured FK once all seven joint positions are available, preventing an initial
jump. A delta arriving before initialization is rejected.

## Build

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-kdl-parser ros-humble-orocos-kdl ros-humble-urdf
mkdir -p ~/ik_ws/src
# clone/copy this repository into ~/ik_ws/src/almost_terminator
cd ~/ik_ws
colcon build --packages-up-to arm_ik_kdl --symlink-install
source install/setup.bash
```

## Run

Provide the same URDF XML to the node's `robot_description` parameter.
For example, when launching alongside robot_state_publisher, pass the parameter
from the same xacro/URDF source (see launch example). The parameter is required.

```bash
ros2 run arm_ik_kdl arm_ik_kdl_node --ros-args \
  -p robot_description:="$(cat /path/to/tiangong_pro_with_hands.urdf)" \
  -p base_link:=waist_yaw_link -p update_rate:=100.0
```

Publish an absolute target:

```bash
ros2 topic pub --once /right_arm/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: waist_yaw_link}, pose: {position: {x: 0.4, y: -0.25, z: 1.0}, orientation: {w: 1.0}}}"
```

Apply a local TCP-frame increment (5 cm forward):

```bash
ros2 topic pub --once /right_arm/target_delta geometry_msgs/msg/TransformStamped \
  "{transform: {translation: {x: 0.05}, rotation: {w: 1.0}}}"
```

Inspect solutions:

```bash
ros2 topic echo /right_arm/ik_solution
```

## Safety and limitations

- Output is diagnostic/planner input only. `JointState` is **not** a motor
  command and must not be connected directly to a hardware command topic.
- This initial implementation does not perform self-collision or environment
  collision checking, velocity/acceleration trajectory generation, or TF
  conversion of incoming targets. Keep it disconnected from actuators until
  those layers are added and validated in simulation.
- KDL's NR_JL solver enforces URDF position limits but does not implement a
  dedicated null-space objective for the redundant seventh DOF.
- The node requires a complete URDF parameter and all seven measured joint
  positions per arm before publishing solutions.
