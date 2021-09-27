import logging
import os
from unittest import TestCase

from rospkg import RosPack

from pycram.knowrob import knowrob


class TestKnowRobWrapper(TestCase):
    def setUp(self) -> None:
        logging.basicConfig(level=logging.INFO)

        self.rospack = RosPack()
        knowrob.clear_beliefstate()
        self.kitchen_owl = os.path.join(self.rospack.get_path('iai_semantic_maps'), "owl", "kitchen.owl")
        self.kitchen_urdf = os.path.join(self.rospack.get_path('pycram'), "resources", "kitchen.urdf")
        self.kitchen_urdf_prefix = "http://knowrob.org/kb/IAI-kitchen.owl#"
        self.kitchen_iri = "http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j"

        self.robot_owl = os.path.join(self.rospack.get_path('knowrob_industrial'), "owl", "UR5Robotiq.owl")
        self.robot_urdf = os.path.join(self.rospack.get_path('pycram'), "resources", "ur5_robotiq.urdf")
        self.robot_iri = "http://knowrob.org/kb/UR5Robotiq.owl#UR5Arm_0"

    def testKbCall(self):
        knowrob.once(f"load_owl('{self.kitchen_owl}')")
        knowrob.once(f"load_owl('{self.robot_owl}')")
        knowrob.once(f"urdf_load('{self.robot_iri}',"
                          f"'{self.robot_urdf}',"
                          f"[load_rdf])")

        res = knowrob.once(
            "kb_call(new_iri(Episode, dul: 'Situation')),"
            "kb_project(is_episode(Episode)),"
            f"kb_project(is_setting_for(Episode,'{self.robot_iri}')),"
            f"kb_project(is_setting_for(Episode,'{self.kitchen_iri}'))")
        self.assertGreater(len(res), 0)

    def testCreateAction(self):
        res = knowrob.neem_create_action()
        self.assertTrue(res.startswith("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_"))

    def testStartEpisode(self):
        act = knowrob.start_episode("soma:'PhysicalTask'", self.kitchen_owl, self.kitchen_iri, self.kitchen_urdf,
                                         self.kitchen_urdf_prefix, self.robot_owl, self.robot_iri, self.robot_urdf)
        self.assertIsNotNone(act)

    def testStopEpisode(self):
        act = knowrob.start_episode(self.kitchen_owl, self.kitchen_iri, self.kitchen_urdf,
                                         self.kitchen_urdf_prefix, self.robot_owl, self.robot_iri, self.robot_urdf)
        knowrob.stop_episode(os.path.join(self.rospack.get_path("pycram"), "neems", "test"))
