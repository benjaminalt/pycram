import logging

import roslibpy
import pybullet as pb

from pycram_ros.rosbridge import ros_client

DIGITAL_TWIN_ENABLED = True


class URJointStateMirror(object):
    def __init__(self, world, joint_state_topic="/ur_digital_twin/joint_state"):
        self.world = world
        self.joint_state_sub = roslibpy.Topic(ros_client, joint_state_topic, "sensor_msgs/JointState")

    def control_simulated_ur(self, joint_state: roslibpy.Message):
        if DIGITAL_TWIN_ENABLED:
            logging.debug(f"URJointStateMirror::control_simulated_ur: {joint_state}")
            joint_indices = list(range(1, 7))
            pb.setJointMotorControlArray(self.world.robot.id, joint_indices, pb.POSITION_CONTROL, targetPositions=joint_state["position"])

    def __enter__(self):
        self.joint_state_sub.subscribe(self.control_simulated_ur)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.joint_state_sub.unsubscribe()