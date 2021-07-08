import logging
import time

from am_control_plugin_python.code_examples.execute_program_synchronously import execute_synchronously
from am_control_plugin_python.data.common_data import PoseQuaternion
from am_control_plugin_python.data.template_data import InsertTemplateRequest, UpdateTemplateInputsRequest, \
    TemplatePorts, TemplatePort
from am_control_plugin_python.rps_interface.http_interface import RPSInterface
from am_control_plugin_python.rps_interface.websocket_interface import RPSWebSocket

from pycram.bullet_world import BulletWorld
from pycram.process_module import ProcessModule
from pycram.process_modules import ProcessModules

import pybullet as pb

from pycram.pycram_industrial.rps_helpers import assert_ok
from pycram.pycram_industrial.rps_scene_mirror import RPSSceneMirror


def pycram_pose_to_rps_pose(pycram_pose) -> PoseQuaternion:
    """
    Convert a pose from PyCRAM's convention (xyz, qxyzw, in meters) to the RPS-internal pose convention (xyz, qwxyz), in mm)
    """
    return PoseQuaternion(x=pycram_pose[0] * 1000,
                          y=pycram_pose[1] * 1000,
                          z=pycram_pose[2] * 1000,
                          qx=pycram_pose[3],
                          qy=pycram_pose[4],
                          qz=pycram_pose[5],
                          qw=pycram_pose[6])


class RPSProcessModule(ProcessModule):
    def __init__(self, rps: RPSInterface = None, bullet_world=None, mirror_scene=True):
        super().__init__()
        self.rps = rps if rps is not None else RPSInterface()
        self.rps.enable_live_data(True)
        rps_ip, rps_port = self.rps.url.lstrip("http://").split(":")
        self.ws = RPSWebSocket(rps_ip, rps_port)
        self.ws.connect()

        self.bullet_world = bullet_world

        self.scene_mirror = None
        if mirror_scene:
            self.scene_mirror = RPSSceneMirror(self.rps, self.bullet_world)
            self.scene_mirror.clear_scene()
            self.scene_mirror.setup_scene()

    def __del__(self):
        self.ws.close()

    def _simulate_compile_execute(self):
        if self.scene_mirror is not None:
            self.scene_mirror.update_scene()    # Update the scene in the RPS for collision-free planning
        assert_ok(self.rps.simulate())
        assert_ok(self.rps.compile())
        if not self.bullet_world:
            # Execute synchronously
            assert_ok(execute_synchronously(self.rps, self.ws))
        else:
            # Execute asynchronously and run PyBullet simulation in parallel until execution finished
            logging.info("RPSProcessModule: Executing with RPS and mirroring in PyBullet...")
            step_pybullet = True

            def notification_callback(noti):
                nonlocal step_pybullet
                if noti["notification"]["type"] == "LiveRunFinished":
                    step_pybullet = False

            assert self.ws.subscribe_notification(notification_callback)
            self.rps.execute_start()

            while step_pybullet:
                pb.stepSimulation()
                time.sleep(1/self.bullet_world.freq)
            logging.info("RPSProcessModule: Execution done.")


class RPSMoveTCP(RPSProcessModule):
    """
    This process moves the tool center point of the arm.
    Equivalent to a 'Move to Point' RPS motion template.
    """
    def _execute(self, desig):
        solution = desig.reference()
        if solution['cmd'] == "move-tcp":
            target = solution['target']  # Goal pose in world coordinates TODO: What is the convention here?
            self.rps.clear_program()
            res = assert_ok(self.rps.insert_template(InsertTemplateRequest("Move to Point")))
            node_id = res["insertedTemplateNodeIds"][0]
            res = assert_ok(self.rps.update_template_inputs(UpdateTemplateInputsRequest(node_id, TemplatePorts(
                poses=[TemplatePort("PointTo", value=PoseQuaternion(target[0] * 1000, target[1] * 1000, target[2] * 1000,
                                                                    *target[3:]))]
            ))))
            self._simulate_compile_execute()


class RPSProcessModules(ProcessModules):
    initialized = None

    def __init__(self, rps: RPSInterface = None, bullet_world: BulletWorld = None, mirror_scene=True):
        if not RPSProcessModules.initialized:
            super().__init__(move_tcp_PM=RPSMoveTCP(rps, bullet_world, mirror_scene))
            RPSProcessModules.initialized = self
