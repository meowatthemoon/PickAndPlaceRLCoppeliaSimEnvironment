from enum import Enum


class ArmActionType(Enum):
    # Absolute arm joint velocities : Sets Joint Target Velocities, steps, then sets them to 0
    ABS_JOINT_VELOCITY = 0

    # Change in arm joint velocities : Gets the current Joint velocities, adds the input, steps, then sets them to 0
    DELTA_JOINT_VELOCITY = 1

    # Absolute arm joint positions/angles (in radians) : Sets Joint Target Positions, steps, then sets them to the current
    ABS_JOINT_POSITION = 2

    # Change in arm joint positions/angles (in radians) : Gets the current Joint Positions, adds the input, steps, then sets them to the current
    DELTA_JOINT_POSITION = 3

    # Absolute arm joint forces/torques : torque_action input, Steps, torque_action with current forces, sets velocities to 0 (torque_action = sets velocities equal to the force if force is negative or mirror if is positive, then sets the forces)
    ABS_JOINT_TORQUE = 4

    # Change in arm joint forces/torques : Gets current forces, adds the input, torque_action the sum, Steps, torque_action with current forces, sets velocities to 0 (torque_action = sets velocities equal to the force if force is negative or mirror if is positive, then sets the forces)
    DELTA_JOINT_TORQUE = 5

    # Absolute end-effector pose (position (3) and quaternion (4)) : ee_action : gets path via jacobian that sets the ee to the desired pose, then performs the path
    ABS_EE_POSE_WORLD_FRAME = 6

    # Change in end-effector pose (position (3) and quaternion (4)) : ee_action : translation current tip pose and input
    DELTA_EE_POSE_WORLD_FRAME = 7

    # Absolute end-effector pose (position (3) and quaternion (4)) : path_action
    # But does path planning between these points
    ABS_EE_POSE_PLAN_WORLD_FRAME = 8

    # ----------------DON'T THINK WE'LL NEED THIS ONE-------------------:
    # Absolute end-effector pose (position (3) and quaternion (4))
    # But does path planning between these points (with collision checking) : path_action
    ABS_EE_POSE_PLAN_WORLD_FRAME_WITH_COLLISION_CHECK = 9

    # Change in end-effector pose (position (3) and quaternion (4)) : path_action : translation current tip pose and input
    # But does path planning between these points
    DELTA_EE_POSE_PLAN_WORLD_FRAME = 10

    # ----------------DON'T THINK WE'LL NEED THIS ONE-------------------:
    # Change in end-effector pose (position (3) and quaternion (4)) : ee_action action relative to the tip : gets path via jacobian that sets the ee to the desired pose, then performs the path
    # In the end-effector frame
    EE_POSE_EE_FRAME = 11

    # ----------------DON'T THINK WE'LL NEED THIS ONE-------------------:
    # Change in end-effector pose (position (3) and quaternion (4)) : __path_action? relative to the tip
    # But does path planning between these points.
    # In the end-effector frame
    EE_POSE_PLAN_EE_FRAME = 12
