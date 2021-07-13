from unittest import TestCase

from rospkg import RosPack

from pycram.knowrob.knowrob_wrapper import KnowRob


class TestKnowRobWrapper(TestCase):
    def setUp(self) -> None:
        self.rospack = RosPack()
        self.knowrob = KnowRob(
            clear_belief_state=True,
            initial_belief_state=f"{self.rospack.get_path('knowrob_industrial')}/misc/mongo_knowrob_default/roslog"
        )
        self.knowrob.once(f"load_owl('{self.rospack.get_path('iai_semantic_maps')}/owl/kitchen.owl')")
        self.knowrob.once(f"load_owl('{self.rospack.get_path('knowrob_industrial')}/owl/UR5Robotiq.owl')")
        self.knowrob.once(f"urdf_load('http://knowrob.org/kb/UR5Robotiq.owl#UR5Arm_0',"
                          f"'{self.rospack.get_path('pycram')}/resources/ur5_robotiq.urdf',"
                          f"[load_rdf])")

    def testKbCall(self):
        res = self.knowrob.once(
            "kb_call(new_iri(Episode, dul: 'Situation')),"
            "kb_project(is_episode(Episode)),"
            "kb_project(is_setting_for(Episode,'http://knowrob.org/kb/UR5Robotiq.owl#UR5Arm_0')),"
            "kb_project(is_setting_for(Episode,'http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j'))")
        self.assertGreater(len(res), 0)
