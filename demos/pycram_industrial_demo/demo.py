import logging
import os

from am_control_plugin_python.rps_interface.http_interface import RPSInterface
from rospkg import RosPack
import pybullet as pb

from demos.pycram_industrial_demo.rps_process_modules import RPSProcessModules
from pycram.bullet_world import BulletWorld, Object
from pycram.knowrob.knowrob_wrapper import KnowRob
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
    pb.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=30, cameraPitch=-70, cameraTargetPosition=[0,0,0])

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

    rospack = RosPack()
    knowrob = KnowRob(clear_belief_state=True, initial_belief_state=os.path.join(rospack.get_path("knowrob_industrial"),
                                                                                 "misc", "mongo_knowrob_default", "roslog"))
    robot_iri = "http://knowrob.org/kb/UR5Robotiq.owl#UR5Arm_0"
    map_iri = "http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j"
    episode_iri = knowrob.neem_init(
        semantic_map_path=os.path.join(rospack.get_path("iai_semantic_maps"), "owl", "kitchen.owl"),
        robot_owl_path=os.path.join(rospack.get_path("knowrob_industrial"), "owl", "UR5Robotiq.owl"),
        robot_urdf_path=os.path.join(rospack.get_path("pycram"), "resources", "ur5_robotiq.urdf"),
        robot_iri=robot_iri,
        map_iri=map_iri
    )

    # Run program
    with tf_broadcaster, jsp, urjsm, fts:
        MotionDesignator(MoveTCPMotionDescription(target=SPAWNING_POSES["cereal"])).perform()

    knowrob.once("tf_logger_disable")

    neem_dir = os.path.join(rospack.get_path("pycram"), "neems", "test_neem")
    if not os.path.exists(neem_dir):
        os.makedirs(neem_dir)
    knowrob.save_neem(neem_dir)
