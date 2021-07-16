from time import time

from pycram.knowrob.knowrob_wrapper import KnowRob


class LogNeemMoveTcp(object):
    def __init__(self, knowrob: KnowRob, parent_act_iri: str, participant_iri: str, robot_iri: str):
        self.knowrob = knowrob
        self.participant_iri = participant_iri
        self.robot_iri = robot_iri
        self.parent_act_iri = parent_act_iri

    def __enter__(self):
        self.begin_act = int(time())
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        end_act = int(time())

        # TODO
        self.knowrob.neem_arm_motion(participant_iri=self.participant_iri,
                                     robot_iri=self.robot_iri,
                                     begin_act=self.begin_act,
                                     end_act=end_act,
                                     parent_act_iri=self.parent_act_iri,
                                     task='AssumingArmPose',
                                     role='MovedObject',
                                     motion='LimbMotion')
