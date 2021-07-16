"""
Adapted from knowrob_refills

https://github.com/refills-project/knowrob_refills/blob/dcb94be9abeb83365efced82c8e1910542e65776/src/knowrob_refills/knowrob_wrapper.py
"""
import logging
import os

from multiprocessing import Lock
from typing import Dict, List, Union

import roslibpy
from rospkg import RosPack

from pycram.knowrob.rosprolog_client import Prolog, PrologException

# from knowrob_industrial.tf_wrapper import lookup_pose

from ros.rosbridge import ROSBridge


class KnowRob(object):
    def __init__(self, initial_belief_state: str = None, clear_beliefstate=False, republish_tf=False, neem_mode=False):
        self.ros_client = ROSBridge().ros_client
        self.prolog = Prolog()

        neem_interface_path = os.path.join(RosPack().get_path("cram_cloud_logger"), "src", "neem-interface.pl")
        self.prolog.once(f"ensure_loaded('{neem_interface_path}')")

        if clear_beliefstate:
            self.clear_beliefstate()
        if initial_belief_state is not None:
            self.load_beliefstate(initial_belief_state)
            logging.info('Loaded initial belief state')

        self.separators = {}
        self.perceived_frame_id_map = {}
        if neem_mode:
            self.load_neem(initial_belief_state)
        self.query_lock = Lock()
        # if (initial_mongo_db is not None or republish_tf) and not neem_mode:
        #     self.republish_tf()
        if neem_mode:
            # self.republish_tf()
            self.new_republish_tf()
        self.order_dict = None

    def once(self, q) -> Union[List, Dict]:
        r = self.all_solutions(q)
        if len(r) == 0:
            return []
        return r[0]

    def all_solutions(self, q):
        logging.info(q)
        r = self.prolog.all_solutions(q)
        return r

    # def pose_to_prolog(self, pose_stamped):
    #     """
    #     :type pose_stamped: PoseStamped
    #     :return: PoseStamped in a form the knowrob likes
    #     :rtype: str
    #     """
    #     if isinstance(pose_stamped, PoseStamped):
    #         return '[\'{}\',[{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
    #                                                            pose_stamped.pose.position.x,
    #                                                            pose_stamped.pose.position.y,
    #                                                            pose_stamped.pose.position.z,
    #                                                            pose_stamped.pose.orientation.x,
    #                                                            pose_stamped.pose.orientation.y,
    #                                                            pose_stamped.pose.orientation.z,
    #                                                            pose_stamped.pose.orientation.w)
    #     elif isinstance(pose_stamped, TransformStamped):
    #         return '[\'{}\', [{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
    #                                                             pose_stamped.transform.translation.x,
    #                                                             pose_stamped.transform.translation.y,
    #                                                             pose_stamped.transform.translation.z,
    #                                                             pose_stamped.transform.rotation.x,
    #                                                             pose_stamped.transform.rotation.y,
    #                                                             pose_stamped.transform.rotation.z,
    #                                                             pose_stamped.transform.rotation.w)
    #
    # def prolog_to_pose_msg(self, query_result):
    #     """
    #     :type query_result: list
    #     :rtype: PoseStamped
    #     """
    #     ros_pose = PoseStamped()
    #     ros_pose.header.frame_id = query_result[0]
    #     ros_pose.pose.position = Point(*query_result[1])
    #     ros_pose.pose.orientation = Quaternion(*query_result[2])
    #     return ros_pose

    def get_mesh(self, object_id):
        q = 'triple(\'{}\', soma:hasShape, S), ' \
            'triple(S,dul:hasRegion,R), ' \
            'triple(R,soma:hasFilePath,P).'.format(object_id)
        solutions = self.once(q)
        if solutions:
            return solutions['P']
        else:
            return None

    def get_object_dimensions(self, object_class):
        """
        :param object_class:
        :return: [x length/depth, y length/width, z length/height]
        """
        q = 'object_dimensions(\'{}\', X_num, Y_num, Z_num).'.format(object_class)
        solutions = self.once(q)
        if solutions:
            return [solutions['Y_num'], solutions['X_num'], solutions['Z_num']]

    def republish_tf(self):
        time = roslibpy.Time.now()
        frame_names = set()
        for i in range(3):
            try:
                q = 'triple(X, knowrob:frameName, Frame), has_type(X, O), transitive(subclass_of(O, dul:\'Object\')).'
                bindings = self.all_solutions(q)
                frame_names_tmp = set()
                for binding in bindings:
                    frame_names.add(str(binding['Frame']))
                if len(frame_names_tmp) > len(frame_names):
                    frame_names = frame_names_tmp
            except:
                logging.warning('failed to get frame names!')
        q = 'forall( member(Frame, {0}), ' \
            '(tf_mng_lookup(Frame, _, {1}.{2}, P, _,_), ' \
            'tf_mem_set_pose(Frame, P, {1}.{2}),!)).'.format(list(frame_names), time.secs, time.nsecs)
        while self.once(q) == []:
            logging.warning('failed to republish tf')
        self.republish_marker()

    def new_republish_tf(self):
        q = 'is_episode(E),triple(E,dul:includesAction,C),time_interval_data(C,Start,End),tf_plugin:tf_republish_set_goal(Start,End).'
        if not self.once(q):
            raise RuntimeError('failed to republish tf')

    def republish_marker(self):
        q = 'marker_plugin:republish'
        self.once(q)

    # def belief_at_update(self, id, pose):
    #     """
    #     :type id: str
    #     :type pose: PoseStamped
    #     """
    #     frame_id = self.get_object_frame_id(id)
    #     q = "get_time(T), " \
    #         "tf_mem_set_pose('{0}', {1}, T), " \
    #         "tf_mng_store('{0}', {1}, T)".format(frame_id,
    #                                              self.pose_to_prolog(pose))
    #     return self.once(q)
        # q = 'is_at(\'{}\', {})'.format(id, self.pose_to_prolog(pose))
        # return self.once(q)

    # def get_objects(self, object_type):
    #     """
    #     Ask knowrob for a specific type of objects
    #     :type object_type: str
    #     :return: all objects of the given type
    #     :rtype: dict
    #     """
    #     objects = OrderedDict()
    #     q = 'instance_of(R, {}).'.format(object_type)
    #     solutions = self.all_solutions(q)
    #     for solution in solutions:
    #         object_id = solution['R'].replace('\'', '')
    #         pose_q = 'belief_at(\'{}\', R).'.format(object_id)
    #         believed_pose = self.once(pose_q)['R']
    #         ros_pose = PoseStamped()
    #         ros_pose.header.frame_id = believed_pose[0]
    #         ros_pose.pose.position = Point(*believed_pose[2])
    #         ros_pose.pose.orientation = Quaternion(*believed_pose[3])
    #         objects[str(object_id)] = ros_pose
    #     return objects

    def get_all_individuals_of(self, object_type):
        q = ' findall(R, instance_of(R, {}), Rs).'.format(object_type)
        solutions = self.once(q)['Rs']
        return [self.remove_quotes(solution) for solution in solutions]

    def remove_quotes(self, s):
        return s.replace('\'', '')

    def get_object_frame_id(self, object_id):
        """
        :type object_id: str
        :return: frame_id of the center of mesh.
        :rtype: str
        """
        q = 'holds(\'{}\', knowrob:frameName, R).'.format(object_id)
        return self.once(q)['R'].replace('\'', '')

    # Neem logging
    def neem_create_action(self):
        """
        Assert a new action into the knowledge base and return its IRI.
        """
        q = "kb_call(new_iri(Action, dul:'Action'))," \
            "kb_project(is_action(Action))"
        solutions = self.all_solutions(q)
        if solutions and len(solutions) > 0:
            return solutions[0]['Action']
        else:
            raise RuntimeError("Failed to create action")

    def neem_log_event(self, act_iri, participant_iri, robot_iri, begin_act, end_act, episode_iri=None,
                       parent_act_iri=None):
        if episode_iri is not None:
            parent = 'is_setting_for(\'{}\',Act)'.format(episode_iri)
        else:
            parent = 'has_subevent(\'{}\',Act)'.format(parent_act_iri)
        q = 'Act = \'{0}\',' \
            'tell([' \
            'has_participant(Act,\'{1}\'),' \
            'is_performed_by(Act,\'{2}\'),' \
            'occurs(Act) during [{3},{4}],' \
            'has_type(RobotRole, soma:\'AgentRole\'),' \
            'has_role(\'{2}\', RobotRole) during Act,' \
            'has_type(Tsk,shop:\'Stocktaking\'),' \
            'has_type(Role,soma:\'Location\'),' \
            'has_task_role(Tsk,Role),' \
            'executes_task(Act,Tsk),' \
            'has_role(\'{5}\',Role) during Act,' \
            '{6}' \
            '])'.format(act_iri, participant_iri, robot_iri, begin_act, end_act, participant_iri, parent)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    def neem_arm_motion(self, participant_iri, robot_iri, begin_act, end_act, parent_act_iri,
                        task="AssumingArmPose", role="MovedObject", motion="LimbMotion"):
        """
        :param participant_iri: The actor performing the action (i.e. the robot arm)
        :param robot_iri: The robot
        :param begin_act: Timestamp of the beginning of the action (in seconds since epoch)
        :param end_act: Timestamp of the end of the action (in seconds since epoch)
        :parent_act_iri: The parent action
        :param role: The role of participant in the context of task
        :param task: The task executed by the action
        :param motion: The motion type classifying the action
        """
        q = f"""kb_project([
                    new_iri(Act, dul:'Action'),
                    is_action(Act),
                    has_participant(Act,'{participant_iri}'),
                    is_performed_by(Act,'{robot_iri}'),
                    new_iri(RobotRole, dul:'Role'),
                    has_type(RobotRole, soma:'AgentRole'),
                    new_iri(Tsk, dul:'Task'),
                    has_type(Tsk,soma:'{task}'),
                    new_iri(Role, dul:'Role'),
                    has_type(Role,soma:'{role}'),
                    has_task_role(Tsk,Role),
                    executes_task(Act,Tsk),
                    new_iri(Mot, soma:'Motion'),
                    has_type(Mot,soma:'{motion}'),
                    is_classified_by(Act,Mot),
                    has_process_role(Mot,Role),
                    holds('{parent_act_iri}',dul:hasConstituent,Act)
                ])"""
        res = self.once(q)
        act_iri = res["Act"]
        robot_role_iri = res["RobotRole"]
        arm_role_iri = res["Role"]

        # Bugfix: kb_project(...) doesn't like variables in occurs, so need to assert this afterward
        # Bugfix: kb_project(...) doesn't like variables in during, so need to assert this afterward
        res = self.once(f"""kb_project([
                                occurs('{act_iri}') during [{begin_act},{end_act}],
                                has_role('{robot_iri}', '{robot_role_iri}') during [{begin_act}, {end_act}],
                                has_role('{participant_iri}', '{arm_role_iri}') during [{begin_act}, {end_act}]
                            ])""")
        return act_iri

    # a
    def neem_park_arm(self, robot_arm_iri, robot_iri, begin_act, end_act, parent_act_iri):
        return self.neem_arm_motion(robot_arm_iri, robot_iri, begin_act, end_act, parent_act_iri,
                                    task='ParkingArms',
                                    role='MovedObject', motion='LimbMotion')

    def neem_navigation(self, participant_iri, robot_iri, begin_act, end_act, parent_act_iri,
                        task, role, motion):
        q = 'tell(is_action(Act)),' \
            'notify_synchronize(event(Act)),' \
            'tell([' \
            'has_participant(Act,\'{0}\'),' \
            'is_performed_by(Act,\'{1}\'),' \
            'occurs(Act) during [{2},{3}],' \
            'has_type(RobotRole, soma:\'AgentRole\'),' \
            'has_role(\'{1}\', RobotRole) during Act,' \
            'has_type(Tsk,soma:\'{5}\'),' \
            'has_type(Role,soma:\'{6}\'),' \
            'executes_task(Act,Tsk),' \
            'has_task_role(Tsk,Role),' \
            'has_type(Mot,soma:\'{7}\'),' \
            'is_classified_by(Act,Mot),' \
            'has_role(\'{0}\',Role) during Act,' \
            'has_subevent(\'{4}\',Act)' \
            '])'.format(participant_iri,  # 0
                        robot_iri,  # 1
                        begin_act,  # 2
                        end_act,  # 3
                        parent_act_iri,  # 4
                        task,  # 5
                        role,  # 6
                        motion)  # 7
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    def load_beliefstate(self, path: str = None):
        logging.info(f"Restoring beliefstate from {path}")
        if path is None:
            self.initial_beliefstate = self.ros_client.get_param('~initial_beliefstate')
        else:
            self.initial_beliefstate = path
        # q = f"remember('{path}')"     # Problem: This requires KnowRob to be initialized
        # self.once(q)
        cmd = f'mongorestore -d roslog {path}'
        if os.system(cmd) != 0:
            raise RuntimeError("Failed to load belief state")

    def clear_beliefstate(self):
        logging.info("Clearing beliefstate")
        self.once("mem_clear_memory")

    def load_neem(self, path, reset_beliefstate=True):
        if reset_beliefstate:
            self.clear_beliefstate()
        self.load_beliefstate(path)

    def save_neem(self, path):
        logging.info(f"Saving NEEM under {path}")
        # q = f'memorize("{path}")'
        # self.once(q)
        self._mongo_dump_database(path, "roslog")

    def load_owl(self, path, ns_alias=None, ns_url=None):
        """
        Example: load_owl("package://knowrob/owl/maps/iai_room_v1.owl", "map", "http://knowrob.org/kb/v1/IAI-Kitchen.owl#")
        :param str path: path to log folder
        :rtype: bool
        """
        if ns_alias is None or ns_url is None:            # Load without namespace
            q = "load_owl('{}')".format(path)
        else:
            q = "load_owl('{0}', [namespace({1},'{2}')])".format(path, ns_alias, ns_url)
        try:
            self.once(q)
            return True
        except PrologException as e:
            logging.warning(e)
            return False

    def start_episode(self, env_owl: str, env_owl_ind_name: str, env_urdf: str, env_urdf_prefix: str,
                      agent_owl: str, agent_owl_ind_name: str, agent_urdf: str):
        """
        Start an episode and return the prolog atom for the corresponding action.
        """
        q = f"mem_episode_start(Action, '{env_owl}', '{env_owl_ind_name}', '{env_urdf}', '{env_urdf_prefix}'," \
            f"'{agent_owl}', '{agent_owl_ind_name}', '{agent_urdf}')"
        res = self.once(q)
        return res["Action"]

    def stop_episode(self, neem_path: str):
        return self.once(f"mem_episode_stop('{neem_path}')")

    def new_iri(self, owl_class: str):
        res = self.once(f"kb_call(new_iri(IRI, {owl_class}))")
        return res["IRI"]

    def _mongo_drop_database(self, name):
        logging.info(f"Dropping database {name}")
        if os.system('mongo {} --eval "db.dropDatabase()"'.format(name)) != 0:
            raise RuntimeError(f"Failed to drop {name}")

    def _mongo_dump_database(self, path, db="roslog"):
        q = f'mng_dump("{db}", "{path}")'
        self.once(q)

    # def start_tf_logging(self):
    #     q = 'ros_logger_start([[\'tf\',[]]])'
    #     self.once(q)
    #
    # def stop_tf_logging(self):
    #     q = 'ros_logger_stop.'
    #     self.once(q)