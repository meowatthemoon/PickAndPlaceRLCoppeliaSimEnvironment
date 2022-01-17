import numpy as np


class Observation:
    def __init__(self,
                 right_shoulder_rgb: np.ndarray,
                 right_shoulder_depth: np.ndarray,
                 overhead_rgb: np.ndarray,
                 overhead_depth: np.ndarray,
                 wrist_rgb: np.ndarray,
                 wrist_depth: np.ndarray,
                 front_rgb: np.ndarray,
                 front_depth: np.ndarray,
                 top_rgb: np.ndarray,
                 top_depth: np.ndarray,
                 joint_velocities: np.ndarray,
                 joint_positions: np.ndarray,
                 joint_forces: np.ndarray,
                 gripper_open: float,
                 gripper_pose: np.ndarray,
                 gripper_touch_forces: np.ndarray,
                 gripper_joint_positions: np.ndarray):
        self.right_shoulder_rgb = right_shoulder_rgb
        self.right_shoulder_depth = right_shoulder_depth
        self.overhead_rgb = overhead_rgb
        self.overhead_depth = overhead_depth
        self.wrist_rgb = wrist_rgb
        self.wrist_depth = wrist_depth
        self.front_rgb = front_rgb
        self.front_depth = front_depth
        self.top_rgb = top_rgb
        self.top_depth = top_depth
        self.joint_velocities = joint_velocities
        self.joint_positions = joint_positions
        self.joint_forces = joint_forces
        self.gripper_open = gripper_open
        self.gripper_pose = gripper_pose
        self.gripper_joint_positions = gripper_joint_positions
        self.gripper_touch_forces = gripper_touch_forces
