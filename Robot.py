import numpy as np
from pyquaternion import Quaternion
from pyrep import PyRep
from pyrep.const import ObjectType
from pyrep.errors import IKError
from pyrep.objects import Dummy, Shape
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

from Action import ArmActionType
from Workspace import Workspace


class Robot:
    def __init__(self, arm_action_type: ArmActionType, object_to_grasp: Shape, environment: PyRep):
        self.arm_action_type = arm_action_type
        self.env = environment
        self.object_to_grasp = object_to_grasp

        self._path_observations = []
        self.target_workspace_check = Dummy.create()

        self.workspace = Workspace()

        # Robot
        self.robot = Panda()
        self._robot_shapes = self.robot.get_objects_in_tree(object_type=ObjectType.SHAPE)

        # Gripper
        self.gripper = PandaGripper()

    def act(self, action):
        # action should contain 1 extra value for gripper open close state
        arm_action = np.array(action[:-1])
        ee_action = action[-1]

        if 0.0 > ee_action > 1.0:
            raise ValueError('Gripper action expected to be within 0 and 1.')

        # Discretize the gripper action
        # study "all" DONE : The all() function returns True if all items in an iterable are true
        open_condition = all(x > 0.9 for x in self.gripper.get_open_amount())
        current_ee = 1.0 if open_condition else 0.0
        if ee_action > 0.5:
            ee_action = 1.0
        elif ee_action < 0.5:
            ee_action = 0.0

        if self.arm_action_type.value == ArmActionType.ABS_JOINT_VELOCITY.value:
            """handles = [sim.simGetObjectHandle(f'Panda_joint{i}') for i in range(1, 8)]
            for i, handle in enumerate(handles):
                sim.simSetJointTargetVelocity(handle, arm_action[i])"""

            self.__assert_action_space(arm_action, (len(self.robot.joints),))
            self.robot.set_joint_target_velocities(arm_action)
            self.env.step()
            self.robot.set_joint_target_velocities(np.zeros_like(arm_action))

        elif self.arm_action_type.value == ArmActionType.DELTA_JOINT_VELOCITY.value:

            self.__assert_action_space(arm_action, (len(self.robot.joints),))
            cur = np.array(self.robot.get_joint_velocities())
            self.robot.set_joint_target_velocities(cur + arm_action)
            self.env.step()
            self.robot.set_joint_target_velocities(np.zeros_like(arm_action))

        elif self.arm_action_type.value == ArmActionType.ABS_JOINT_POSITION.value:

            self.__assert_action_space(arm_action, (len(self.robot.joints),))
            self.robot.set_joint_target_positions(arm_action)
            self.env.step()
            self.robot.set_joint_target_positions(self.robot.get_joint_positions())

        elif self.arm_action_type.value == ArmActionType.DELTA_JOINT_POSITION.value:

            self.__assert_action_space(arm_action, (len(self.robot.joints),))
            cur = np.array(self.robot.get_joint_positions())
            self.robot.set_joint_target_positions(cur + arm_action)
            self.env.step()
            self.robot.set_joint_target_positions(self.robot.get_joint_positions())

        elif self.arm_action_type.value == ArmActionType.ABS_JOINT_TORQUE.value:

            self.__assert_action_space(arm_action, (len(self.robot.joints),))
            self.__torque_action(arm_action)
            self.env.step()
            self.__torque_action(self.robot.get_joint_forces())
            self.robot.set_joint_target_velocities(np.zeros_like(arm_action))

        elif self.arm_action_type.value == ArmActionType.DELTA_JOINT_TORQUE.value:

            cur = np.array(self.robot.get_joint_forces())
            new_action = cur + arm_action
            self.__torque_action(new_action)
            self.env.step()
            self.__torque_action(self.robot.get_joint_forces())
            self.robot.set_joint_target_velocities(np.zeros_like(arm_action))

        elif self.arm_action_type.value == ArmActionType.ABS_EE_POSE_WORLD_FRAME.value:

            self.__assert_action_space(arm_action, (7,))
            self.__ee_action(list(arm_action))

        elif self.arm_action_type.value == ArmActionType.ABS_EE_POSE_PLAN_WORLD_FRAME.value:

            self.__assert_action_space(arm_action, (7,))
            self._path_observations = []
            self._path_observations = self.__path_action(
                list(arm_action), collision_checking=False)

        elif self.arm_action_type.value == ArmActionType.ABS_EE_POSE_PLAN_WORLD_FRAME_WITH_COLLISION_CHECK.value:

            self.__assert_action_space(arm_action, (7,))
            self._path_observations = []
            self._path_observations = self.__path_action(
                list(arm_action), collision_checking=True)

        elif self.arm_action_type.value == ArmActionType.DELTA_EE_POSE_PLAN_WORLD_FRAME.value:

            self.__assert_action_space(arm_action, (7,))
            a_x, a_y, a_z, a_qx, a_qy, a_qz, a_qw = arm_action
            x, y, z, qx, qy, qz, qw = self.robot.get_tip().get_pose()
            new_rot = Quaternion(a_qw, a_qx, a_qy, a_qz) * Quaternion(qw, qx,
                                                                      qy, qz)
            qw, qx, qy, qz = list(new_rot)
            new_pose = [a_x + x, a_y + y, a_z + z] + [qx, qy, qz, qw]
            self._path_observations = []
            self._path_observations = self.__path_action(list(new_pose))

        elif self.arm_action_type.value == ArmActionType.DELTA_EE_POSE_WORLD_FRAME.value:

            self.__assert_action_space(arm_action, (7,))
            a_x, a_y, a_z, a_qx, a_qy, a_qz, a_qw = arm_action
            x, y, z, qx, qy, qz, qw = self.robot.get_tip().get_pose()
            new_rot = Quaternion(a_qw, a_qx, a_qy, a_qz) * Quaternion(
                qw, qx, qy, qz)
            qw, qx, qy, qz = list(new_rot)
            new_pose = [a_x + x, a_y + y, a_z + z] + [qx, qy, qz, qw]
            self.__ee_action(list(new_pose))

        elif self.arm_action_type.value == ArmActionType.EE_POSE_EE_FRAME.value:

            self.__assert_action_space(arm_action, (7,))
            self.__ee_action(
                list(arm_action), relative_to=self.robot.get_tip())

        elif self.arm_action_type.value == ArmActionType.EE_POSE_PLAN_EE_FRAME.value:
            self.__assert_action_space(arm_action, (7,))
            self._path_observations = []
            self._path_observations = self.__path_action(list(arm_action), relative_to=self.robot.get_tip())

        else:
            raise RuntimeError(f'Unrecognised action mode: {self.arm_action_type}')

        # study and fix each of these DONE
        if current_ee != ee_action:
            done_closing = False
            if ee_action == 0.0:  # and self._attach_grasped_objects
                # If gripper close action, the check for grasp.
                self.gripper.grasp(self.object_to_grasp)
            else:
                # If gripper open action, the check for ungrasp.
                self.gripper.release()
            while not done_closing:
                done_closing = self.gripper.actuate(ee_action, velocity=0.2)
                self.env.step()
            if ee_action == 1.0:
                # Step a few more times to allow objects to drop
                for _ in range(10):
                    self.env.step()

    @staticmethod
    def __assert_action_space(action, expected_shape):
        if np.shape(action) != expected_shape:
            raise RuntimeError(
                'Expected the action shape to be: %s, but was shape: %s' % (
                    str(expected_shape), str(np.shape(action))))

    @staticmethod
    def __assert_unit_quaternion(quat):
        if not np.isclose(np.linalg.norm(quat), 1.0):
            raise RuntimeError('Action contained non unit quaternion!')

    def __check_target_in_workspace(self, target_pos: np.ndarray) -> bool:

        workspace_minx, workspace_maxx, workspace_miny, workspace_maxy, workspace_minz, workspace_maxz = self.workspace.get_bounding_box()

        x, y, z = target_pos
        return (workspace_maxx > x > workspace_minx and
                workspace_maxy > y > workspace_miny and
                workspace_maxz > z > workspace_minz)

    def __ee_action(self, action, relative_to=None):
        self.__assert_unit_quaternion(action[3:])
        try:
            joint_positions = self.robot.solve_ik_via_jacobian(
                action[:3], quaternion=action[3:], relative_to=relative_to)
            self.robot.set_joint_target_positions(joint_positions)
        except IKError as e:
            raise Exception(
                'Could not perform IK via Jacobian. This is because the current'
                ' end-effector pose is too far from the given target pose. '
                'Try limiting your action space, or sapping to an alternative '
                'action mode, e.g. ABS_EE_POSE_PLAN_WORLD_FRAME') from e
        done = False
        prev_values = None
        # Move until reached target joint positions or until we stop moving
        # (e.g. when we collide wth something)
        while not done:
            self.env.step()
            cur_positions = self.robot.get_joint_positions()
            reached = np.allclose(cur_positions, joint_positions, atol=0.01)
            not_moving = False
            if prev_values is not None:
                not_moving = np.allclose(
                    cur_positions, prev_values, atol=0.001)
            prev_values = cur_positions
            done = reached or not_moving

    def __path_action(self, action, collision_checking=False, relative_to=None):
        self.__assert_unit_quaternion(action[3:])
        # Check if the target is in the workspace; if not, then quick reject
        # Only checks position, not rotation
        pos_to_check = action[:3]
        if relative_to is not None:
            self.target_workspace_check.set_position(
                pos_to_check, relative_to)
            pos_to_check = self.target_workspace_check.get_position()
        valid = self.__check_target_in_workspace(pos_to_check)
        if not valid:
            raise Exception('Target is outside of workspace.')

        observations = []
        done = False
        if collision_checking:
            # First check if we are colliding with anything
            colliding = self.robot.check_arm_collision()
            if colliding:
                # Disable collisions with the objects that we are colliding with
                grasped_objects = self.gripper.get_grasped_objects()
                colliding_shapes = [s for s in self.env.get_objects_in_tree(
                    object_type=ObjectType.SHAPE) if (
                                            s.is_collidable() and
                                            s not in self._robot_shapes and
                                            s not in grasped_objects and
                                            self.robot.check_arm_collision(s))]
                [s.set_collidable(False) for s in colliding_shapes]
                path = self.__path_action_get_path(
                    action, collision_checking, relative_to)
                [s.set_collidable(True) for s in colliding_shapes]
                # Only run this path until we are no longer colliding
                while not done:
                    done = path.step()
                    self.env.step()
                    colliding = self.robot.check_arm_collision()
                    if not colliding:
                        break
                    """
                    success = self.__success()
                    # If the task succeeds while traversing path, then break early
                    if success:
                        done = True
                        break
                    """
        if not done:
            path = self.__path_action_get_path(
                action, collision_checking, relative_to)
            while not done:
                done = path.step()
                self.env.step()

        return observations

    def __path_action_get_path(self, action, collision_checking, relative_to):
        try:
            path = self.robot.get_path(
                action[:3], quaternion=action[3:],
                ignore_collisions=not collision_checking,
                relative_to=relative_to,
            )
            return path
        except IKError as e:
            raise Exception('Could not find a path.') from e

    def __torque_action(self, action):
        _TORQUE_MAX_VEL = 9999
        # If force is negative, apply positive velocity, otherwise, apply negative velocity
        self.robot.set_joint_target_velocities(
            [(_TORQUE_MAX_VEL if t < 0 else -_TORQUE_MAX_VEL)
             for t in action])
        self.robot.set_joint_forces(np.abs(action))
