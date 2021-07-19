import logging
import os

from am_control_plugin_python.rps_interface.http_interface import RPSInterface
from rospkg import RosPack
import pybullet as pb

from demos.pycram_industrial_demo.rps_process_modules import RPSProcessModules
from pycram.bullet_world import BulletWorld, Object
from pycram.knowrob.knowrob_wrapper import KnowRob
from pycram.pycram_industrial.neem_logging import LogNeemMoveTcp
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
    "robot": [0.1, 0.1, 0.89, 0.0, 0.0, 0.0, 1.0],   # x,y,z,qx,qy,qz,qw
    "motherboard": [0.6, 0.4, 0.89, 0.0, 0.0, 0.0, 1.0],
    "ram_stick": [0.6, 0.6, 0.9, 0.0, 0.0, 0.0, 1.0]
}


if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    world = BulletWorld()
    pb.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=30, cameraPitch=-70, cameraTargetPosition=[0,0,0])
    world.set_gravity([0, 0, -9.8])

    # Load environment, robot and objects
    rospack = RosPack()
    lab_urdf_path = os.path.join(rospack.get_path("knowrob_industrial"), "urdf", "maps",
                                 "motherboard_experiment_lab.urdf")
    robot_urdf_path = os.path.join(rospack.get_path("knowrob_industrial"), "urdf", "robots", "ur5_robotiq.urdf")
    motherboard_urdf_path = os.path.join(rospack.get_path("knowrob_industrial"), "urdf", "objects", "motherboard.urdf")
    ram_urdf_path = os.path.join(rospack.get_path("knowrob_industrial"), "urdf", "objects",
                                 "ddr3-4Gbyte-ram-module-kingston.urdf")

    plane = Object("floor", "environment", os.path.join(RESOURCE_DIR, "plane.urdf"), world=world)
    lab = Object("lab", "environment", lab_urdf_path)
    robot = Object("ur", "robot", robot_urdf_path,
                   position=SPAWNING_POSES["robot"][:3], orientation=SPAWNING_POSES["robot"][3:])
    motherboard = Object("motherboard", "object", motherboard_urdf_path,
                         position=SPAWNING_POSES["motherboard"][:3], orientation=SPAWNING_POSES["motherboard"][3:])
    ram_stick = Object("ram_stick", "object", ram_urdf_path,
                       position=SPAWNING_POSES["ram_stick"][:3], orientation=SPAWNING_POSES["ram_stick"][3:])
    BulletWorld.robot = robot

    tf_broadcaster = TFBroadcaster(world, "map", "odom", "projection", interval=1.0)
    jsp = JointStatePublisher(world)
    urjsm = URJointStateMirror(world)
    fts = ForceTorqueSensor(world, "ee_fixed_joint")

    # Setup process modules
    rps = RPSInterface("nb067")
    ProcessModule.resolvers.append(lambda desig: RPSProcessModules(rps, world).initialized.available_process_modules(desig))

    knowrob = KnowRob(clear_beliefstate=True)
    robot_iri = "http://knowrob.org/kb/UR5Robotiq.owl#UR5Robotiq_0"
    arm_iri = "http://knowrob.org/kb/UR5Robotiq.owl#UR5Arm_0"
    map_iri = "http://knowrob.org/kb/motherboard_experiment_lab.owl#motherboard_experiment_lab"

    # Run program
    top_level_action = knowrob.start_episode(env_owl=os.path.join(rospack.get_path("knowrob_industrial"), "owl", "motherboard_experiment_lab.owl"),
                                             env_owl_ind_name=map_iri,
                                             env_urdf=lab_urdf_path,
                                             env_urdf_prefix="http://knowrob.org/kb/motherboard_experiment_lab.owl#",
                                             agent_owl=os.path.join(rospack.get_path("knowrob_industrial"), "owl", "UR5Robotiq.owl"),
                                             agent_owl_ind_name=robot_iri,
                                             agent_urdf=robot_urdf_path)

    with tf_broadcaster, jsp, urjsm, fts:
        with LogNeemMoveTcp(knowrob, parent_act_iri=top_level_action, participant_iri=arm_iri, robot_iri=robot_iri):
            MotionDesignator(MoveTCPMotionDescription(target=SPAWNING_POSES["ram_stick"])).perform()

    neem_dir = os.path.join(rospack.get_path("pycram"), "neems", "test")
    if not os.path.exists(neem_dir):
        os.makedirs(neem_dir)

    knowrob.stop_episode(neem_dir)
