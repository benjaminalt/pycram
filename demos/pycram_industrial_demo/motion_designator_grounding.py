from pycram.motion_designator import *

"""
The grounding functions for the Motion designator descriptions in pycram/motion_designator.py
They all infer missing properties and return a dictionary with the properties as value.
"""


def ground_move_tcp(self):  # Nothing to ground here
    if not self.arm:
        self.arm = 'left'
    self._check_properties("[Motion Designator] Move-TCP")
    return self.__dict__


MoveTCPMotionDescription.ground = ground_move_tcp


def call_ground(desig):
    return [desig._description.ground()]


MotionDesignator.resolvers.append(call_ground)
