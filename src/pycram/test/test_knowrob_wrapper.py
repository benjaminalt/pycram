import logging
import os
from unittest import TestCase

from rospkg import RosPack

from pycram.knowrob.knowrob_wrapper import KnowRob


class TestKnowRobWrapper(TestCase):
    def setUp(self) -> None:
        logging.basicConfig(level=logging.INFO)

        self.rospack = RosPack()
        self.knowrob = KnowRob(clear_beliefstate=True)  #,initial_belief_state=f"{self.rospack.get_path('knowrob_industrial')}/misc/mongo_knowrob_default/roslog"
        self.kitchen_owl = os.path.join(self.rospack.get_path('iai_semantic_maps'), "owl", "kitchen.owl")
        self.kitchen_urdf = os.path.join(self.rospack.get_path('pycram'), "resources", "kitchen.urdf")
        self.kitchen_urdf_prefix = "http://knowrob.org/kb/IAI-kitchen.owl#"
        self.kitchen_iri = "http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j"

        self.robot_owl = os.path.join(self.rospack.get_path('knowrob_industrial'), "owl", "UR5Robotiq.owl")
        self.robot_urdf = os.path.join(self.rospack.get_path('pycram'), "resources", "ur5_robotiq.urdf")
        self.robot_iri = "http://knowrob.org/kb/UR5Robotiq.owl#UR5Arm_0"

    def testKbCall(self):
        self.knowrob.once(f"load_owl('{self.kitchen_owl}')")
        self.knowrob.once(f"load_owl('{self.robot_owl}')")
        self.knowrob.once(f"urdf_load('{self.robot_iri}',"
                          f"'{self.robot_urdf}',"
                          f"[load_rdf])")

        res = self.knowrob.once(
            "kb_call(new_iri(Episode, dul: 'Situation')),"
            "kb_project(is_episode(Episode)),"
            f"kb_project(is_setting_for(Episode,'{self.robot_iri}')),"
            f"kb_project(is_setting_for(Episode,'{self.kitchen_iri}'))")
        self.assertGreater(len(res), 0)

    def testCreateAction(self):
        res = self.knowrob.neem_create_action()
        self.assertTrue(res.startswith("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_"))

    def testStartEpisode(self):
        act = self.knowrob.start_episode("soma:'PhysicalTask'", self.kitchen_owl, self.kitchen_iri, self.kitchen_urdf,
                                         self.kitchen_urdf_prefix, self.robot_owl, self.robot_iri, self.robot_urdf)
        self.assertIsNotNone(act)

    def testStopEpisode(self):
        act = self.knowrob.start_episode(self.kitchen_owl, self.kitchen_iri, self.kitchen_urdf,
                                         self.kitchen_urdf_prefix, self.robot_owl, self.robot_iri, self.robot_urdf)
        self.knowrob.stop_episode(os.path.join(self.rospack.get_path("pycram"), "neems", "test"))
