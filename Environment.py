import os
from typing import Union, List

import numpy as np
from pyrep import PyRep

from Action import ArmActionType
from Camera import Camera
from Context import Context, Color, Position
from Observation import Observation
from Robot import Robot


class Environment:
    def __init__(self, arm_action_type: ArmActionType, num_boxes: int, goal_box_idx: int, max_step_count: int = 100,
                 box_positions: List[Position] = None, stack_position: Position = None,
                 background_color: Color = Color.Transparent):
        self.max_step_count = max_step_count

        # Launch CoppeliaSim
        if arm_action_type == ArmActionType.ABS_JOINT_VELOCITY or arm_action_type == ArmActionType.DELTA_JOINT_VELOCITY:
            ttt_file = os.path.join(
                "/home/andre/Documents/PhD/Environment/Scenes/rlbench_task_design_no_control_loop.ttt")
        else:
            ttt_file = os.path.join("/home/andre/Documents/PhD/Environment/Scenes/rlbench_task_design.ttt")

        self.env = PyRep()
        self.env.launch(ttt_file, responsive_ui=True)
        # Context : Spawns the boxes and stack
        self.context = Context(num_boxes=num_boxes, goal_box_idx=goal_box_idx, background_color=background_color,
                               box_positions=box_positions, stack_position=stack_position)

        # Cameras : Creates the references to each camera
        self.cam_over_shoulder_right = Camera('cam_over_shoulder_right')
        self.cam_overhead = Camera('cam_overhead')
        self.cam_wrist = Camera('cam_wrist')
        self.cam_front = Camera('cam_front')
        self.cam_top = Camera('cam_top')

        # Robot
        self.robot = Robot(arm_action_type=arm_action_type,
                           environment=self.env,
                           object_to_grasp=self.context.boxes[self.context.goal_box_idx].object)

        # Initial values
        self.step_counter = 0
        self._reset_called = False

    def close(self):
        self.env.stop()
        self.env.shutdown()

    def reset(self) -> Observation:
        self.env.stop()

        self.context.reset()

        self.step_counter = 0
        self._reset_called = True

        self.env.start()
        self.env.step()

        return self.get_observation()

    def step(self, action) -> (Observation, int, bool):
        if not self._reset_called:
            raise RuntimeError(
                "Call 'reset' before calling 'step' on a task.")

        self.robot.act(action)

        done = self.success()
        task_reward = self.__reward()
        reward = float(done) if task_reward is None else task_reward

        return self.get_observation(), reward, done

    def get_observation(self) -> Observation:
        tip = self.robot.robot.get_tip()

        fs = self.robot.robot.get_joint_forces()
        vels = self.robot.robot.get_joint_target_velocities()
        #   by default the noise is identity, hence returns the value it receives, doing nothing to it
        #   the values are: f if velocity is positive, -f if velocity is negative
        joint_forces = np.array([-f if v < 0 else f for f, v in zip(fs, vels)])

        ee_forces = self.robot.gripper.get_touch_sensor_forces()
        ee_forces_flat = []
        # Don't think this does anything, only warns that extend is deprecated,
        # then again do we need this touch sensor forces?
        for eef in ee_forces:
            ee_forces_flat.extend(eef)
        ee_forces_flat = np.array(ee_forces_flat)

        right_shoulder_rgb, right_shoulder_depth = self.cam_over_shoulder_right.get_rgb_depth()
        overhead_rgb, overhead_depth = self.cam_overhead.get_rgb_depth()
        wrist_rgb, wrist_depth = self.cam_wrist.get_rgb_depth()
        front_rgb, front_depth = self.cam_front.get_rgb_depth()
        top_rgb, top_depth = self.cam_top.get_rgb_depth()

        gripper_open = 1.0 if self.robot.gripper.get_open_amount()[0] > 0.9 else 0.0

        obs = Observation(
            right_shoulder_rgb=right_shoulder_rgb,
            right_shoulder_depth=right_shoulder_depth,
            overhead_rgb=overhead_rgb,
            overhead_depth=overhead_depth,
            wrist_rgb=wrist_rgb,
            wrist_depth=wrist_depth,
            front_rgb=front_rgb,
            front_depth=front_depth,
            top_rgb=top_rgb,
            top_depth=top_depth,
            joint_velocities=np.array(self.robot.robot.get_joint_velocities()),
            joint_positions=np.array(self.robot.robot.get_joint_positions()),
            joint_forces=joint_forces,
            gripper_open=gripper_open,
            gripper_pose=np.array(tip.get_pose()),
            gripper_touch_forces=ee_forces_flat,
            gripper_joint_positions=np.array(self.robot.gripper.get_joint_positions())
        )
        return obs

    @staticmethod
    def __reward() -> Union[float, None]:
        return None

    def success(self) -> bool:
        """
        Returns TRUE only if BOX is FULLY inside STACK AND on TOP of it and TOUCHING it.
        :return: boolean
        """
        stack_position = self.context.stack.get_position()
        box_position = self.context.boxes[self.context.goal_box_idx].get_position()

        box_w = self.context.box_size / 2
        stack_w = self.context.stack_size / 2

        if (stack_position.x + stack_w) > (box_position.x - box_w) >= (stack_position.x - stack_w) \
                and (stack_position.y + stack_w) > (box_position.y - box_w) >= (stack_position.y - stack_w) \
                and (stack_position.x + stack_w) > (box_position.x + box_w) >= (stack_position.x - stack_w) \
                and (stack_position.y + stack_w) > (box_position.y + box_w) >= (stack_position.y - stack_w) \
                and (box_position.z - stack_position.z <= 0.045):
            return True
        return False


if __name__ == "__main__":
    for i in range(10):
        env = Environment(arm_action_type=ArmActionType.ABS_JOINT_POSITION, num_boxes=3, goal_box_idx=0,
                          background_color=Color.Black)

        env.reset()

        print(f"final box 1 : {str(env.context.boxes[0].get_position())}")
        print(f"final box 2 : {str(env.context.boxes[1].get_position())}")
        print(f"final box 3 : {str(env.context.boxes[2].get_position())}")
        print(f"final stack : {str(env.context.stack.get_position())}")
        print(f"final bg plane : {str(env.context.background_plane.get_position())}")

        input("ok?" + str(i + 1))

        env.close()
