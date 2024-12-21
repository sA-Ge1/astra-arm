from typing import List

# Move groups for Marc arm
MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

# Optional prefix for joint/link naming
prefix: str = ""
# Joint names for Marc's arm
def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "shoulder_joint",
        prefix + "upperarm_joint",
        prefix + "elbow_joint",
        prefix + "extra_wrist_joint",
        prefix + "wrist_joint",
        prefix + "ee_rotation_joint_x"
    ]

# Base link name
def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"

# End effector name
def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "ee_rotation_x"

# Gripper joint names
def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "gripper_joint_1", 
        prefix + "gripper_joint_2"
    ]
