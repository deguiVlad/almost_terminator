"""Motor ID mapping for the TienKung 2.0 Pro arm joints."""

_ARM_JOINTS = (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow_pitch", "elbow_yaw", "wrist_pitch", "wrist_roll",
)

MOTOR_IDS = {}
for _index, _joint in enumerate(_ARM_JOINTS):
    MOTOR_IDS[f"{_joint}_l_joint"] = 11 + _index
    MOTOR_IDS[f"{_joint}_r_joint"] = 21 + _index


def motor_id_for(joint_name: str) -> int:
    """Return the configured actuator ID or raise for an unknown joint."""
    try:
        return MOTOR_IDS[joint_name]
    except KeyError as exc:
        raise ValueError(f"No TienKung motor ID mapping for joint {joint_name!r}") from exc
