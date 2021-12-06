from unittest import TestCase

import numpy as np

from pycram.knowrob import knowrob
from pycram.location_designator import ObjectRelativeLocationDesignatorDescription, LocationDesignator, \
    LocationDesignatorDescription
from pycram.object_designator import LocatedObjectDesignatorDescription, ObjectDesignator
from pycram.resolver import location_designator_grounding     # do not remove
from pycram.resolver import object_designator_grounding       # do not remove


class TestLocationDesignatorGrounding(TestCase):
    def setUp(self) -> None:
        knowrob.clear_beliefstate()
        knowrob.once("""
            kb_project([is_individual(object1), instance_of(object1, objectType)]),
            tf_logger_enable,
            mem_tf_set(object1, [world, [1.0, 2.0, 0.5], [0.0, 0.0, 0.0, 1.0]], 4.0)""")

    def tearDown(self) -> None:
        knowrob.clear_beliefstate()

    def test_ground_object_relative_location(self):
        """
        Test grounding of ObjectRelativeLocationDesignator (generation of absolute poses in world frame, given an
        object and a relative transformation
        """
        object_desig = ObjectDesignator(LocatedObjectDesignatorDescription(name="object1"))
        desc = ObjectRelativeLocationDesignatorDescription(relative_pose=[0.2, 0.1, 1.3, 0.0, 0.0, 0.0, 1.0],
                                                           reference_object=object_desig)
        desig = LocationDesignator(desc)
        sol = desig.reference()
        pose_world = sol["pose"]
        self.assertTrue(np.allclose(np.array(pose_world), np.array([1.2, 2.1, 1.8, 0.0, 0.0, 0.0, 1.0])))

    def test_ground_location(self):
        """
        Test default grounding of LocationDesignator
        """
        desc = LocationDesignatorDescription(pose=[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
        desig = LocationDesignator(desc)
        sol = desig.reference()
        pose_world = sol["pose"]
        self.assertTrue(np.allclose(np.array(pose_world), np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])))
