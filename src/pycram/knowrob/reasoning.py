import json
from typing import List

from neem_interface_python.rosprolog_client import atom

from pycram.knowrob.knowrob_wrapper import KnowRob
from pycram.knowrob.utils import knowrob_string_to_pose


def object_pose(knowrob: KnowRob, object_iri: str, reference_cs: str = None) -> List[float]:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    :param reference_cs: The coordinate system relative to which the pose should be defined
    """
    if reference_cs is None:
        obj_pose = knowrob.once(f"tf_get_pose({atom(object_iri)}, Pose)")["Pose"]
        return knowrob_string_to_pose(obj_pose)
    else:
        res = knowrob.once(f"tf_get_pose({atom(object_iri)}, [{atom(reference_cs)}, Pos, Ori])")
        pos = json.loads(res["Pos"])
        ori = json.loads(res["Ori"])
        return pos + ori
