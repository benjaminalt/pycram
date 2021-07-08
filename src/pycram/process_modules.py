from .robot_description import InitializedRobotDescription as robot_description
from .motion_designator import *



class ProcessModules():

    def __init__(self, navigation_PM=None, pick_up_PM=None, place_PM=None, accessing_PM=None, park_arms_PM=None,
                 looking_PM=None, opening_gripper_PM=None, closing_gripper_PM=None, detecting_PM=None, move_tcp_PM=None,
                 move_joints_PM=None, world_state_detecting_PM=None):
        self.navigation = navigation_PM
        self.pick_up = pick_up_PM
        self.place = place_PM
        self.accessing = accessing_PM
        self.park_arms = park_arms_PM
        self.move_head = looking_PM
        self.opening_gripper = opening_gripper_PM
        self.closing_gripper = closing_gripper_PM
        self.detecting = detecting_PM
        self.move_tcp = move_tcp_PM
        self.move_joints = move_joints_PM
        self.world_state_detecting = world_state_detecting_PM

    def available_process_modules(self, desig):
        """
        This method chooses the right process module for the given designator and returns it.
        :param desig: The designator for which a process module should be choosen.
        :return: The choosen process module
        """
        if type(desig._description) == MoveMotionDescription:
            return self.navigation

        if type(desig._description) == PickUpMotionDescription:
            return self.pick_up

        if type(desig._description) == PlaceMotionDescription:
            return self.place

        if type(desig._description) == AccessingMotionDescription:
            return self.accessing

        #if desig.check_constraints([('type', 'park-arms')]):
        #    return self.park_arms

        if type(desig._description) == LookingMotionDescription:
            return self.move_head

        if type(desig._description) == MoveGripperMotionDescription:
            return self.opening_gripper

        if type(desig._description) == MoveGripperMotionDescription:
            return self.closing_gripper

        if type(desig._description) == DetectingMotionDescription:
            return self.detecting

        if type(desig._description) == MoveTCPMotionDescription:
            return self.move_tcp

        if type(desig._description) == MoveArmJointsMotionDescription:
            return self.move_joints

        if type(desig._description) == WorldStateDetectingMotionDescription:
            return self.world_state_detecting
