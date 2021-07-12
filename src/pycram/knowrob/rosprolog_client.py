"""
Rosprolog client loosely coupled to ROS and compatible with Python 3
"""

import json
from enum import Enum

import roslibpy

from ros.rosbridge import ROSBridge


class PrologException(Exception):
    pass


class PrologNextSolutionResponse(Enum):
    NO_SOLUTION = 0
    WRONG_ID = 1
    QUERY_FAILED = 2
    OK = 3


class Upper(object):
    def __init__(self, iterable):
        self._iter = iter(iterable)

    def __next__(self):  # Py3-style iterator interface
        return next(self._iter)  # builtin next() function calls

    def __iter__(self):
        return self


class PrologQuery(object):
    def __init__(self, query_str: str, simple_query_srv: roslibpy.Service, next_solution_srv: roslibpy.Service,
                 finish_srv: roslibpy.Service, iterative=True):
        """
        This class wraps around the different rosprolog services to provide a convenient python interface.
        :param iterative: if False, all solutions will be calculated by rosprolog during the first service call
        """
        self._simple_query_srv = simple_query_srv
        self._next_solution_srv = next_solution_srv
        self._finish_query_srv = finish_srv

        self._finished = False
        self._query_id = None
        result = self._simple_query_srv.call(roslibpy.ServiceRequest({"id": self.get_id(), "query": query_str,
                                                                      "mode": 1 if iterative else 0}))
        if not result["ok"]:
            raise PrologException('Prolog query failed: {}'.format(result["message"]))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.finish()

    def solutions(self):
        """
        :rtype: Iterator[dict]
        """
        try:
            while not self._finished:
                next_solution = self._next_solution_srv.call(roslibpy.ServiceRequest({"id": self.get_id()}))
                if next_solution["status"] == PrologNextSolutionResponse.OK.value:  # Have to compare to .value here because roslibpy msg does not have types
                    yield self._json_to_dict(next_solution["solution"])
                elif next_solution["status"] == PrologNextSolutionResponse.WRONG_ID.value:
                    raise PrologException(
                        f'Query id {self.get_id()} invalid. Maybe another process terminated our query?')
                elif next_solution["status"] == PrologNextSolutionResponse.QUERY_FAILED.value:
                    raise PrologException(f'Prolog query failed: {next_solution["solution"]}')
                elif next_solution["status"] == PrologNextSolutionResponse.NO_SOLUTION.value:
                    break
                else:
                    raise PrologException(f'Unknown query status {next_solution["solution"]}')
        finally:
            self.finish()

    def finish(self):
        if not self._finished:
            try:
                self._finish_query_srv.call(roslibpy.ServiceRequest({"id": self.get_id()}))
            finally:
                self._finished = True

    def get_id(self):
        """
        :rtype: str
        """
        if self._query_id is None:
            self._query_id = 'PYTHON_QUERY_{}'.format(roslibpy.Time.now().to_nsec())
        return self._query_id

    def _json_to_dict(self, json_text):
        """
        :type json_text: str
        :rtype: dict
        """
        return json.loads(json_text)


class Prolog(object):
    def __init__(self, name_space='rosprolog'):
        """
        :type name_space: str
        :param timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        :type timeout: int
        """
        ros_host, ros_port = ROSBridge.get_ros_master_host_and_port()
        self.ros_client = roslibpy.Ros(ros_host, ros_port)
        self.ros_client.run()

        self._simple_query_srv = roslibpy.Service(self.ros_client, f'{name_space}/query', "json_prolog_msgs/srv/PrologQuery")
        self._next_solution_srv = roslibpy.Service(self.ros_client, f'{name_space}/next_solution', "json_prolog_msgs/srv/PrologNextSolution")
        self._finish_query_srv = roslibpy.Service(self.ros_client, f'{name_space}/finish', "json_prolog_msgs/srv/PrologFinish")

    def __del__(self):
        self.ros_client.terminate()

    def query(self, query_str):
        """
        Returns an Object which asks rosprolog for one solution at a time.
        :type query_str: str
        :rtype: PrologQuery
        """
        return PrologQuery(query_str, simple_query_srv=self._simple_query_srv,
                           next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)

    def once(self, query_str):
        """
        Call rosprolog once and finished it.
        :type query_str: str
        :rtype: list
        """
        q = None
        try:
            q = PrologQuery(query_str, simple_query_srv=self._simple_query_srv,
                            next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)
            return next(Upper(q.solutions()))
        except StopIteration:
            return []
        finally:
            if q is not None:
                q.finish()

    def all_solutions(self, query_str):
        """
        Requests all solutions from rosprolog, this might take a long time!
        :type query_str: str
        :rtype: list
        """
        return list(PrologQuery(query_str,
                                iterative=False,
                                simple_query_srv=self._simple_query_srv,
                                next_solution_srv=self._next_solution_srv,
                                finish_srv=self._finish_query_srv).solutions())
