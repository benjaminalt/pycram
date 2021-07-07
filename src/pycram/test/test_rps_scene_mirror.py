import os

from am_control_plugin_python.rps_interface.http_interface import RPSInterface

from pycram.bullet_world import BulletWorld, Object
from pycram.pycram_industrial.rps_scene_mirror import RPSSceneMirror

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
RESOURCES_DIR = os.path.join(SCRIPT_DIR, os.pardir, os.pardir, os.pardir, "resources")

if __name__ == '__main__':
    world = BulletWorld()
    plane = Object("floor", "environment", os.path.join(RESOURCES_DIR, "plane.urdf"), world=world)
    kitchen = Object("kitchen", "environment", os.path.join(RESOURCES_DIR, "kitchen.urdf"), world=world)
    rps = RPSInterface("nb067")
    scene_mirror = RPSSceneMirror(rps, world)
    scene_mirror.clear_scene()
    scene_mirror.setup_scene()
    scene_mirror.update_scene()