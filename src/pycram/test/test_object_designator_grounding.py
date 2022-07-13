import os
from copy import deepcopy
from unittest import TestCase, skip

from pathlib import Path

import numpy as np
from pycram.knowrob import knowrob
from pycram.bullet_world import BulletWorld, Object
from pycram.object_designator import LocatedObjectDesignatorDescription, ObjectDesignator
from pycram.resolver import object_designator_grounding_knowrob     # do not remove

SCRIPT_DIR = Path(__file__).parent
RESOURCES_DIR = SCRIPT_DIR / "resources"

class TestObjectDesignatorGrounding(TestCase):
    def setUp(self) -> None:
        self.world = BulletWorld(type="GUI")
        self.obj_a = Object("a", type="box", path=(RESOURCES_DIR / "box.urdf").as_posix(), position=[1.0, 1.0, 1.0],
               orientation=[0.1399279, 0.6029767, 0.1279054, 0.7749061], world=self.world)
        self.obj_b = Object("b", type="cylinder", path=(RESOURCES_DIR / "cylinder.urdf").as_posix(), position=[2.0, 2.0, 2.0],
               orientation=[0.4619398, 0.1913417, 0.4619398, 0.7325378], world=self.world)
        self.objects = [self.obj_a, self.obj_b]
        knowrob.clear_beliefstate()
        for obj in self.objects:
            knowrob.once(f"""
                kb_project([is_individual({obj.name}), instance_of({obj.name}, {obj.type})]),
                tf_logger_enable,
                mem_tf_set({obj.name}, [world, {list(obj.get_position())}, {list(obj.get_orientation())}], 4.0),
                tf_logger_disable""")

    def tearDown(self) -> None:
        knowrob.clear_beliefstate()

    def test_ground_located_object_by_type(self):
        """
        Test grounding of ObjectDesignator when just the type is given.
        Expected behavior: Repeated calls to desig.next_solution() successively yield groundings for all known objects
        of that type
        """
        desc = LocatedObjectDesignatorDescription(type_="box")
        desig = ObjectDesignator(desc)
        all_solutions = self._get_all_solutions(desig)
        names = [sol["name"] for sol in all_solutions]
        self.assertSetEqual(set(names), {"a"})
        for sol in all_solutions:
            if sol["name"] == "a":
                self.assertTrue(np.allclose(sol["pose"], list(self.obj_a.get_position() + self.obj_a.get_orientation())))

    def test_ground_obb(self):
        desc = LocatedObjectDesignatorDescription(name="a")
        desig = ObjectDesignator(desc)
        all_solutions = self._get_all_solutions(desig)
        obj = all_solutions[0]
        self.world.add_vis_axis((obj["pose"][:3], obj["pose"][3:]), length=0.2, delete_existing=False)
        obb = obj["obb"]
        # for point in obb:
        #     self.world.add_vis_axis((point[:3], point[3:]), length=0.1, delete_existing=False)
        self.assertTrue(np.allclose(obb[0], [0.9409534243953696, 0.9383758674624709, 1.0146958565032365, 0.1399279018814937, 0.6029767081077246, 0.12790540171983725, 0.7749061104195154]))
        self.assertTrue(np.allclose(obb[1], [1.0590465756046306, 1.061624132537529, 0.9853041434967637, 0.1399279018814937, 0.6029767081077246, 0.12790540171983725, 0.7749061104195154]))

    @staticmethod
    def _get_all_solutions(desig):
        all_solutions = []
        while True:
            res = desig.reference()
            all_solutions.append(deepcopy(res))
            desig = desig.next_solution()
            if desig is None:
                break
        return all_solutions
