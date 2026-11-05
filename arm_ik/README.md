# arm_ik — TienKung 2.0 Pro arm IK (ROS 2 Humble)

Python package for independent 7-DOF left/right arm inverse kinematics.

## Model source

The runtime model is parsed from a URDF supplied through `urdf_path` or the
transient-local `/robot_description` topic (`std_msgs/msg/String`). The URDF
provides joint origins, axes, limits and fixed transforms to the TCP. The node
does not infer joint axes from dynamic TF: current TF transforms already contain
the joint rotations and cannot reliably recover URDF axes.

Base link defaults to `waist_yaw_link`. The expected ordered joints are
`shoulder_pitch`, `shoulder_roll`, `shoulder_yaw`, `elbow_pitch`,
`elbow_yaw`, `wrist_pitch`, `wrist_roll`, with `_l_joint` / `_r_joint`
suffixes. Preferred TCP links are `left_tcp_link` and `right_tcp_link`;
legacy `L_base_link` and `R_base_link` are also accepted.

The node remains in a non-ready state until both valid 7-DOF models are built.
It will not move using the approximate offline test model or TF-only geometry.

## Build and test

```bash
source /opt/ros/humble/setup.bash
cd <ros2_workspace>
colcon build --packages-select arm_ik --symlink-install
source install/setup.bash
```

Offline tests (from the package root):

```bash
python3 -m pytest test -q
```

## Launch

Use the robot's published `/robot_description`:

```bash
ros2 launch arm_ik arm_ik.launch.py use_sim_time:=true
```

Or supply the URDF explicitly:

```bash
ros2 launch arm_ik arm_ik.launch.py urdf_path:=/absolute/path/to/robot.urdf
```

The robot/simulator must also publish `/joint_states` and TF. TF2 transforms
incoming goals into `base_frame`; it is not the source of joint-axis geometry.

## Interfaces

### Inputs

| Topic | Type | Meaning |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Measured joint positions. |
| `/arm_ik/goal_left`, `/arm_ik/goal_right` | `geometry_msgs/msg/PoseStamped` | Absolute TCP pose. |
| `/arm_ik/delta_left`, `/arm_ik/delta_right` | `geometry_msgs/msg/TwistStamped` | Relative TCP displacement. |

For `PoseStamped`, `header.frame_id` is transformed to `base_frame`.
An empty frame ID is interpreted as already being in `base_frame`.

For `TwistStamped`, `twist.linear` is displacement in metres, not velocity;
`twist.angular` is a rotation vector in radians, not angular velocity.
`delta_rotation_frame` selects base or current TCP axes.

### Outputs

| Topic | Type | Meaning |
|---|---|---|
| `/arm/cmd_pos` | `bodyctrl_msgs/msg/CmdSetMotorPosition` | Motor position commands, only if `bodyctrl_msgs` is installed. |
| `/arm_ik/wrist_left_pose`, `/arm_ik/wrist_right_pose` | `geometry_msgs/msg/PoseStamped` | Measured TCP poses from FK. |
| `/arm_ik/left/status`, `/arm_ik/right/status` | `std_msgs/msg/String` | JSON execution state and pose errors. |

## Examples

Absolute pose:

```bash
ros2 topic pub --once /arm_ik/goal_right geometry_msgs/msg/PoseStamped \
"{header: {frame_id: waist_yaw_link}, pose: {position: {x: 0.45, y: -0.20, z: 1.0}, orientation: {w: 1.0}}}"
```

Relative +5 cm along base X:

```bash
ros2 topic pub --once /arm_ik/delta_right geometry_msgs/msg/TwistStamped \
"{header: {frame_id: waist_yaw_link}, twist: {linear: {x: 0.05}}}"
```

Built-in demos:

```bash
ros2 launch arm_ik arm_ik.launch.py demo_goals:=true
ros2 launch arm_ik arm_ik.launch.py demo_delta:=true delta_side:=right delta_x:=0.05
```

## Key parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `base_frame` | `waist_yaw_link` | Goal and output frame. |
| `kinematics_base_frame` | `waist_yaw_link` | URDF chain root. |
| `urdf_path` | empty | Optional URDF file; otherwise wait for `/robot_description`. |
| `rate_hz` | 50 | Control-loop rate. |
| `joint_state_timeout` | 0.5 s | Stop publishing commands when feedback is older than this. |
| `spd_limit` | 0.8 rad/s | Command speed cap (never above 0.8). |
| `a_max` | 0.4 rad/s² | Trajectory acceleration cap. |
| `tol_pos` / `tol_rot` | 0.01 m / 0.05 rad | Goal tolerances. |
| `enable_table_guard` | false | Enable analytic table-plane checks. |
| `h_table` / `table_margin` | 0.9 m / 0.02 m | Table height and margin in guard frame. |
| `waypoint_lift` | 0.25 m | Raised intermediate waypoint. |
| `n_seeds` | 12 | Maximum IK seeds. |
| `delta_reference` | measured | `measured` or `last_goal`. |
| `delta_accumulate` | true | Accumulate consecutive delta goals. |
| `delta_rotation_frame` | base | `base` or `tcp`. |
| `delta_max_lin` / `delta_max_rot` | 0.35 m / 1.2 rad | Single-command delta limits. |

## Safety and limitations

- Do not connect to real actuators until URDF geometry, motor IDs, motor signs,
  limits, frame calibration and the `bodyctrl_msgs` command schema have been
  independently verified on the target robot.
- Without `bodyctrl_msgs`, the node computes IK but does not publish motor
  commands; this is a dry-run.
- The table guard is a plane approximation, not full self-collision or
  environment collision checking. It is disabled by default and must be
  calibrated before enabling.
- Numerical IK can reject unreachable or poorly conditioned targets. For absolute
  goals only, an unreachable orientation may be relaxed while preserving the
  position target.
- The node requires fresh, complete `/joint_states` snapshots; stale feedback
  moves active execution to `failed` and stops publishing motor commands.
- `srv/SolveIK.srv` is currently only a source definition; this ament_python
  package does not generate it. The supported public interface is topics.
- Offline tests do not certify ROS 2 integration or real-hardware safety.
