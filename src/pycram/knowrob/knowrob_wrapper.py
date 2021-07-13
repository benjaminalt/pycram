"""
Adapted from knowrob_refills

https://github.com/refills-project/knowrob_refills/blob/dcb94be9abeb83365efced82c8e1910542e65776/src/knowrob_refills/knowrob_wrapper.py
"""
import logging
import os

from multiprocessing import Lock

import roslibpy

from pycram.knowrob.rosprolog_client import Prolog, PrologException

# from knowrob_industrial.tf_wrapper import lookup_pose

from ros.rosbridge import ROSBridge


class KnowRob(object):
    def __init__(self, initial_belief_state: str = None, clear_belief_state=False, republish_tf=False, neem_mode=False):
        self.ros_client = ROSBridge().ros_client
        self.prolog = Prolog()

        if initial_belief_state is not None:
            if clear_belief_state:
                self._mongo_drop_database('roslog')
                logging.info("Cleared belief state")
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

    def once(self, q):
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
    # helper to create an action
    def neem_create_action(self):
        q = 'tell(is_action(Act))'
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]['Act']

    def neem_init(self, semantic_map_path: str = "package://knowrob_refills/owl/iai-shop.owl",
                  robot_owl_path: str = "package://knowrob/owl/robots/IIWA.owl",
                  robot_urdf_path: str = "package://knowrob/urdf/iiwa.urdf",
                  robot_iri: str = "http://knowrob.org/kb/IIWA.owl#IIWA_0",
                  map_iri: str = "http://knowrob.org/kb/iai-shop.owl#IAIShop_0"):
        q = "tf_logger_enable," \
            f"load_owl('{semantic_map_path}')," \
            f"load_owl('{robot_owl_path}')," \
            f"urdf_load('{robot_iri}', '{robot_urdf_path}', [load_rdf])," \
            "kb_call(new_iri(Episode, dul: 'Situation'))," \
            "kb_project(is_episode(Episode))," \
            f"kb_project(is_setting_for(Episode,'{robot_iri}'))," \
            f"kb_project(is_setting_for(Episode,'{map_iri}'))"
        solutions = self.all_solutions(q)
        if solutions is None or len(solutions) == 0:
            raise RuntimeError("Failed to initialize NEEM")
        return solutions[0]['Episode']

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
                        task, role, motion):
        """
        For an example of this in use, see LogNeemArmMotion in knowrob_refills knowrob_wrapper.py
        """
        q = 'tell(is_action(Act)),' \
            'notify_synchronize(event(Act)),' \
            'tell([' \
            'has_participant(Act,\'{0}\'),' \
            'is_performed_by(Act,\'{1}\'),' \
            'occurs(Act) during [{2},{3}],' \
            'has_type(RobotRole, soma:\'AgentRole\'),'\
            'has_role(\'{1}\', RobotRole) during Act,'\
            'has_type(Tsk,soma:\'{5}\'),' \
            'has_type(Role,soma:\'{6}\'),' \
            'has_task_role(Tsk,Role),' \
            'has_role(\'{0}\',Role) during Act,' \
            'executes_task(Act,Tsk),' \
            'has_type(Mot,soma:\'{7}\'),' \
            'is_classified_by(Act,Mot),' \
            'has_process_role(Mot,Role),' \
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

    def reset_beliefstate(self, path: str = None):
        self._mongo_drop_database('roslog')
        self.load_beliefstate(path)

    def load_neem(self, path, reset_beliefstate=True):
        if reset_beliefstate:
            self.reset_beliefstate(path)
        else:
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

    def start_episode(self):
        raise NotImplementedError()

    #     self.clear_beliefstate(path_to_old_episode)
    #     q = 'tell([is_episode(Episode)]).'
    #     result = self.once(q)
    # if path_to_old_episode is not None:
    #     q = 'remember({})'.format(path_to_old_episode)
    #     result = self.once(q)
    #     if result == []:
    #         raise RuntimeError('failed to load {}'.format(path_to_old_episode))
    # if result:
    #     q = 'knowrob_memory:current_episode(E), mem_episode_stop(E)'
    #     self.once(q)

    # if path_to_old_episode is None:
    #     q = 'mem_episode_start(E).'
    #     result = self.once(q)
    #     self.episode_id = result['E']
    # else:
    #     q = 'mem_episode_start(E, [import:\'{}\']).'.format(path_to_old_episode)
    #     result = self.once(q)
    #     self.episode_id = result['E']
    # return result != []

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