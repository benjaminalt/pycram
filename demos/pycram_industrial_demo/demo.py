import logging
import os

from am_control_plugin_python.rps_interface.http_interface import RPSInterface

from demos.pycram_industrial_demo.rps_process_modules import RPSProcessModules
from pycram.bullet_world import BulletWorld, Object
from pycram.robot_description import InitializedRobotDescription as robot_description
import motion_designator_grounding
from pycram.motion_designator import *
from ros.force_torque_sensor import ForceTorqueSensor
from ros.joint_state_publisher import JointStatePublisher
from ros.tf_broadcaster import TFBroadcaster
from ros.ur_joint_state_mirror import URJointStateMirror

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PYCRAM_DIR = os.path.join(SCRIPT_DIR, os.pardir, os.pardir)
RESOURCE_DIR = os.path.join(PYCRAM_DIR, "resources")

SPAWNING_POSES = {
    "robot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "cereal": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
}


if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    world = BulletWorld()
    world.set_gravity([0, 0, -9.8])
    plane = Object("floor", "environment", os.path.join(RESOURCE_DIR, "plane.urdf"), world=world)

    robot = Object("ur", "robot", os.path.join(RESOURCE_DIR, "ur5_robotiq.urdf"),
                   position=SPAWNING_POSES["robot"][:3], orientation=SPAWNING_POSES["robot"][3:])
    kitchen = Object("kitchen", "environment", os.path.join(RESOURCE_DIR, "kitchen.urdf"))
    cereal = Object("cereal", "cereal", os.path.join(RESOURCE_DIR, "breakfast_cereal.stl"),
                    position=SPAWNING_POSES["cereal"][:3], orientation=SPAWNING_POSES["cereal"][3:])
    BulletWorld.robot = robot

    tf_broadcaster = TFBroadcaster(world, "map", "odom", "projection", "iai_kitchen", interval=1.0)
    jsp = JointStatePublisher(world)
    urjsm = URJointStateMirror(world)
    fts = ForceTorqueSensor(world, "ee_fixed_joint")

    # Setup process modules
    rps = RPSInterface("nb067")
    ProcessModule.resolvers.append(lambda desig: RPSProcessModules(rps, world).initialized.available_process_modules(desig))

    # Program
    with tf_broadcaster, jsp, urjsm, fts:
        MotionDesignator(MoveTCPMotionDescription(target=SPAWNING_POSES["cereal"])).perform()
