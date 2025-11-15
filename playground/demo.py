import os

from semantic_digital_twin.adapters.urdf import  URDFParser
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.world import World
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import Context
from pycram.testing import setup_world

world = setup_world()
pr2_view = PR2.from_world(world)

context = Context(world, pr2_view)

