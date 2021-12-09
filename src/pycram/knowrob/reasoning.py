from typing import List, Union

import numpy as np
from pytransform3d.rotations import quaternion_wxyz_from_xyzw, quaternion_xyzw_from_wxyz
from pytransform3d.transformations import transform_from_pq, pq_from_transform

from neem_interface_python.rosprolog_client import atom

from pycram.knowrob import knowrob
from pycram.knowrob.utils import knowrob_string_to_pose
from pycram.object_designator import ObjectDesignator


def object_type(object_iri: str) -> str:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    """
    return knowrob.once(f"kb_call(instance_of({atom(object_iri)}, Class))")["Class"]


def instances_of(type_: str) -> List[str]:
    """
    :param type_: An object type (i.e. class)
    """
    all_sols = knowrob.all_solutions(f"kb_call(instance_of(Individual, {atom(type_)}))")
    return [sol["Individual"] for sol in all_sols]


def object_pose(object_iri: str, reference_cs: str = "world", timestamp=None) -> List[float]:
    """
    :param object_iri: The name (identifier) of the object individual in the KnowRob knowledge base
    :param reference_cs: The coordinate system relative to which the pose should be defined
    """
    if timestamp is None:
        res = knowrob.once(f"mem_tf_get({atom(object_iri)}, [{atom(reference_cs)}, Pos, Ori])")
    else:
        res = knowrob.once(f"mem_tf_get({atom(object_iri)}, [{atom(reference_cs)}, Pos, Ori], {timestamp})")
    pos = res["Pos"]
    ori = res["Ori"]
    return pos + ori


def grasp_pose(object_iri: str) -> List[float]:
    query = f"""
    kb_call(has_grasp_point({atom(object_iri)}, GraspPointName)),
    mem_tf_get(GraspPointName, world, Pos, Ori)
    """
    res = knowrob.once(query)
    pos = res["Pos"]
    ori = res["Ori"]
    return pos + ori


def feature_pose(obj: Union[dict, ObjectDesignator], feature_name: str) -> List[float]:
    object_grounded = obj.reference() if type(obj) == ObjectDesignator else obj
    object_iri = object_grounded["name"]
    reference_frame, feature_frame_pos, feature_frame_ori = knowrob.once(f"""
        urdf_link_parent_joint({atom(object_iri)}, {atom(feature_name)}, FeatureFixedJoint),
        urdf_joint_origin({atom(object_iri)}, FeatureFixedJoint, FeatureFrameRelative)
    """)["FeatureFrameRelative"]
    feature_frame_relative_pq = np.concatenate((feature_frame_pos, quaternion_wxyz_from_xyzw(feature_frame_ori)))

    res = knowrob.once(f"mem_tf_get({atom(object_iri)}, [world, Pos, Ori])")
    world_T_obj_pq = np.concatenate((res["Pos"], quaternion_wxyz_from_xyzw(res["Ori"])))
    obj_T_world = transform_from_pq(world_T_obj_pq)
    feature_T_obj = transform_from_pq(feature_frame_relative_pq)
    feature_T_world = feature_T_obj @ obj_T_world
    feature_frame_world_pq = pq_from_transform(feature_T_world)
    feature_frame_world = np.concatenate((feature_frame_world_pq[:3], quaternion_xyzw_from_wxyz(feature_frame_world_pq[3:])))
    return feature_frame_world.tolist()
