"""
Adapted from knowrob_refills

https://github.com/refills-project/knowrob_refills/blob/dcb94be9abeb83365efced82c8e1910542e65776/src/knowrob_refills/knowrob_wrapper.py
"""
import logging

from typing import Dict, List, Union

from neem_interface_python.rosprolog_client import Prolog, PrologException
from neem_interface_python.neem_interface import NEEMInterface


neem_interface = NEEMInterface()
prolog = Prolog()


def all_solutions(q):
    logging.info(q)
    r = prolog.all_solutions(q)
    return r


def once(q) -> Union[List, Dict]:
    r = all_solutions(q)
    if len(r) == 0:
        return []
    return r[0]


def get_mesh(object_id):
    """
    TODO: Untested
    """
    q = 'triple(\'{}\', soma:hasShape, S), ' \
        'triple(S,dul:hasRegion,R), ' \
        'triple(R,soma:hasFilePath,P).'.format(object_id)
    solutions = once(q)
    if solutions:
        return solutions['P']
    else:
        return None


def get_object_dimensions(object_class):
    """
    TODO: Untested
    :param object_class:
    :return: [x length/depth, y length/width, z length/height]
    """
    q = 'object_dimensions(\'{}\', X_num, Y_num, Z_num).'.format(object_class)
    solutions = once(q)
    if solutions:
        return [solutions['Y_num'], solutions['X_num'], solutions['Z_num']]


def get_all_individuals_of(object_type):
    q = ' findall(R, instance_of(R, {}), Rs).'.format(object_type)
    solutions = once(q)['Rs']
    return [remove_quotes(solution) for solution in solutions]


def remove_quotes(s):
    return s.replace('\'', '')


def load_beliefstate(path: str):
    logging.info(f"Restoring beliefstate from {path}")
    once(f"remember('{path}')")


def clear_beliefstate():
    logging.info("Clearing beliefstate")
    once("mem_clear_memory")


def load_owl(path, ns_alias=None, ns_url=None):
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
        once(q)
        return True
    except PrologException as e:
        logging.warning(e)
        return False


def new_iri(owl_class: str):
    res = once(f"kb_call(new_iri(IRI, {owl_class}))")
    return res["IRI"]
