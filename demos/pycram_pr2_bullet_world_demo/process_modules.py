from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pycram.bullet_world import BulletWorld
from pycram.helper import transform
import pycram.bullet_world_reasoning as btr
import pybullet as p
import numpy as np
import time

ik_joints = ["fl_caster_rotation_joint", "fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
            "fr_caster_rotation_joint", "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
            "bl_caster_rotation_joint", "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
            "br_caster_rotation_joint", "br_caster_l_wheel_joint", "br_caster_r_wheel_joint",
            "head_pan_joint", "head_tilt_joint", "laser_tilt_mount_joint", "r_shoulder_pan_joint",
            "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint",
            "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint",
            "r_gripper_motor_slider_joint", "r_gripper_motor_screw_joint",
            "r_gripper_l_finger_joint", "r_gripper_l_finger_tip_joint",
            "r_gripper_r_finger_joint", "r_gripper_r_finger_tip_joint",
            "r_gripper_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint",
            "l_wrist_flex_joint", "l_wrist_roll_joint", "l_gripper_motor_slider_joint",
            "l_gripper_motor_screw_joint", "l_gripper_l_finger_joint",
            "l_gripper_l_finger_tip_joint", "l_gripper_r_finger_joint",
            "l_gripper_r_finger_tip_joint", "l_gripper_joint", "torso_lift_motor_screw_joint"]


def _apply_ik(robot, joint_poses):
    """
    Apllies a list of joint poses calculated by an inverse kinematics solver to a robot
    :param robot: The robot the joint poses should be applied on
    :param joint_poses: The joint poses to be applied
    :return: None
    """
    for i in range(0, len(ik_joints)):
        robot.set_joint_state(ik_joints[i], joint_poses[i])


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of the PR2 and applies them to the, in the BulletWorld
    defined robot.
    :return:
    """
    #joint_poses = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.9, -0.1, 1.6, 1.7,
    #               0.087, 1.2, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9, -0.1, 1.6,
    #               -1.7, -0.08, -1.2, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    robot = BulletWorld.robot
    if arm == "right":
        for joint, pose in robot_description.i.get_static_joint_chain("right", "park").items():
            robot.set_joint_state(joint, pose)
    if arm == "left":
        for joint, pose in robot_description.i.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class Pr2Navigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            robot = BulletWorld.robot
            robot.set_position_and_orientation(solution['target'], solution['orientation'])

class Pr2PickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'pick':
            object = solution['object']
            robot = BulletWorld.robot
            target = object.get_position()
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), target,
                                               maxNumIterations=100)
            _apply_ik(robot, inv)
            robot.attach(object, solution['gripper'])
            time.sleep(0.5)


class Pr2Place(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'place':
            object = solution['object']
            robot = BulletWorld.robot
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(solution['gripper']), solution['target'],
                                           maxNumIterations=100)
            _apply_ik(robot, inv)
            robot.detach(object)
            time.sleep(0.5)


class Pr2Accessing(ProcessModule):
    """
    This process module responsible for opening drawers to access the objects inside. This works by firstly moving
    the end effector to the handle of the drawer. Next, the end effector is moved the respective distance to the back.
    This provides the illusion the robot would open the drawer by himself.
    Then the drawer will be opened by setting the joint pose of the drawer joint.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == 'access':
            kitchen = solution['part-of']
            robot = BulletWorld.robot
            gripper = solution['gripper']
            drawer_handle = solution['drawer-handle']
            drawer_joint = solution['drawer-joint']
            dis = solution['distance']
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), kitchen.get_link_position(drawer_handle))
            _apply_ik(robot, inv)
            time.sleep(0.2)
            han_pose = kitchen.get_link_position(drawer_handle)
            new_p = [han_pose[0] - dis, han_pose[1], han_pose[2]]
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), new_p)
            _apply_ik(robot, inv)
            print(drawer_joint)
            kitchen.set_joint_state(drawer_joint, 0.3)
            time.sleep(0.5)


class Pr2ParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """
    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class Pr2MoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """
    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'looking':
            target = solutions['target']
            robot = BulletWorld.robot
            pose_in_pan = transform(target, robot.get_link_position("head_pan_link"))
            pose_in_tilt = transform(target, robot.get_link_position("head_tilt_link"))

            new_pan = np.arctan([pose_in_pan[1], pose_in_pan[0]])
            new_tilt = np.arctan([-pose_in_tilt[2], pose_in_tilt[0]**2 + pose_in_tilt[1]**2])

            robot.set_joint_state("head_pan_joint", new_pan[0])
            robot.set_joint_state("head_tilt_joint", new_tilt[0])


class Pr2MoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-gripper":
            robot = BulletWorld.robot
            gripper = solution['gripper']
            motion = solution['motion']
            for joint, state in robot_description.i.get_static_gripper_chain(gripper, motion).items():
                # TODO: Test this, add gripper-opening/-closing to the demo.py
                robot.set_joint_state(joint, state)
            time.sleep(0.5)


class Pr2Detecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "detecting":
            robot = BulletWorld.robot
            object_type = solution['object']
            cam_frame_name = solution['cam_frame']
            front_facing_axis = solution['front_facing_axis']

            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)
            for obj in objects:
                if btr.visible(obj, robot.get_link_position_and_orientation(cam_frame_name), front_facing_axis):
                    return obj


class Pr2MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-tcp":
            target = solution['target']
            gripper = solution['gripper']
            robot = BulletWorld.robot
            inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper), target)
            _apply_ik(robot, inv)
            time.sleep(0.5)


class Pr2MoveJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-joints":
            robot = BulletWorld.robot
            right_arm_poses = solution['right-poses']
            left_arm_poses = solution['left-poses']
            if type(right_arm_poses) == dict:
                for joint, pose in right_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(right_arm_poses) == str and right_arm_poses == "park":
                _park_arms("right")

            if type(left_arm_poses) == dict:
                for joint, pose in left_arm_poses.items():
                    robot.set_joint_state(joint, pose)
            elif type(right_arm_poses) == str and left_arm_poses == "park":
                _park_arms("left")

            time.sleep(0.5)


class Pr2WorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "world-state-detecting":
            obj_type = solution['object']
            return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


pr2_navigation = Pr2Navigation()
pr2_pick_up = Pr2PickUp()
pr2_place = Pr2Place()
pr2_accessing = Pr2Accessing()
pr2_park_arms = Pr2ParkArms()
pr2_move_head = Pr2MoveHead()
pr2_move_gripper = Pr2MoveGripper()
pr2_detecting = Pr2Detecting()
pr2_move_tcp = Pr2MoveTCP()
pr2_move_joints = Pr2MoveJoints()
pr2_world_state_detecting = Pr2WorldStateDetecting()


def available_process_modules(desig):
    """
    This method chooses the right process module for the given designator and returns it.
    :param desig: The designator for which a process module should be choosen.
    :return: The choosen process module
    """
    if desig.check_constraints([('type', 'moving')]):
        return pr2_navigation

    if desig.check_constraints([('type', 'pick-up')]):
        return pr2_pick_up

    if desig.check_constraints([('type', 'place')]):
        return pr2_place

    if desig.check_constraints([('type', 'accessing')]):
        return pr2_accessing

    if desig.check_constraints([('type', 'park-arms')]):
        return pr2_park_arms

    if desig.check_constraints([('type', 'looking')]):
        return pr2_move_head

    if desig.check_constraints([('type', 'opening-gripper')]):
        return pr2_move_gripper

    if desig.check_constraints([('type', 'closing-gripper')]):
        return pr2_move_gripper

    if desig.check_constraints([('type', 'detecting')]):
        return pr2_detecting

    if desig.check_constraints([('type', 'move-tcp')]):
        return pr2_move_tcp

    if desig.check_constraints([('type', 'move-arm-joints')]):
        return pr2_move_joints

    if desig.check_constraints([('type', 'world-state-detecting')]):
        return pr2_world_state_detecting


ProcessModule.resolvers.append(available_process_modules)
