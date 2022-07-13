import time
from typing import List

from neem_interface_python.rosprolog_client import atom

from pycram.bullet_world import BulletWorld
from pycram.designator import DesignatorError
from pycram.helper import pycram_pose_from_affine, affine_from_pycram_pose
from pycram.knowrob import knowrob
from pycram.knowrob.reasoning import object_type, instances_of
from pycram.knowrob.utils import pose_to_knowrob_string
from pycram.object_designator import LocatedObjectDesignatorDescription, ObjectDesignator


def _update_object_pose_in_knowrob(object_iri: str, pose: List[float]):
    pose_str = pose_to_knowrob_string(pose)
    knowrob.once(f"mem_tf_set({atom(object_iri)}, {pose_str}, {time.time()})")


def _ground_pose(desc: LocatedObjectDesignatorDescription):
    if len(BulletWorld.current_bullet_world.get_objects_by_name(desc.name)) == 0:
        raise DesignatorError(f"No object with name {desc.name} in Bullet world")
    pos, ori = BulletWorld.current_bullet_world.get_objects_by_name(desc.name)[0].get_position_and_orientation()
    desc.pose = pos + ori


def _ground_aabb(desc: LocatedObjectDesignatorDescription):
    if len(BulletWorld.current_bullet_world.get_objects_by_name(desc.name)) == 0:
        raise DesignatorError(f"No object with name {desc.name} in Bullet world")
    desc.aabb = BulletWorld.current_bullet_world.get_objects_by_name(desc.name)[0].get_aabb()


def _ground_obb(desc: LocatedObjectDesignatorDescription):
    """
    Compute the oriented bounding box of an object using a trick:
    1. Move the object to be aligned with the world origin
    2. Compute its AABB
    3. Move the AABB to the object's actual pose
    """
    objects_with_matching_name = BulletWorld.current_bullet_world.get_objects_by_name(desc.name)
    if len(objects_with_matching_name) == 0:
        raise DesignatorError(f"No object with name {desc.name} in Bullet world")
    obj = objects_with_matching_name[0]
    obj_position, obj_orientation = obj.get_position_and_orientation()
    obj.set_position_and_orientation([0, 0, 0], [0, 0, 0, 1])
    aabb_at_origin = obj.get_aabb()
    obj.set_position_and_orientation(obj_position, obj_orientation)
    obb = []
    world_T_obj = affine_from_pycram_pose(obj_position + obj_orientation)
    for point in aabb_at_origin:
        world_T_point = affine_from_pycram_pose(point)
        point_transformed_affine = world_T_obj @ world_T_point
        obb.append(pycram_pose_from_affine(point_transformed_affine))
    desc.obb = obb


def ground_located_object(description: LocatedObjectDesignatorDescription):
    if not description.type and not description.name:
        raise RuntimeError("Could not ground LocatedObjectDesignatorDescription: Either type or name must be given")
    if not description.type:
        description.type = object_type(description.name)
    if not description.name:
        # Retrieve all objects of the given type, fetch their poses and yield the grounded description
        print(f"Locating objects of type {description.type}")
        object_names = instances_of(description.type)
        for object_name in object_names:
            print(f"Locating object pose for {object_name}")
            description.name = object_name
            _ground_pose(description)
            if description.aabb is None:
                _ground_aabb(description)
            if description.obb is None:
                _ground_obb(description)
            _update_object_pose_in_knowrob(description.name, description.pose)
            yield description.__dict__
        raise StopIteration()
    elif not description.pose:
        # Fetch the object pose and yield the grounded description
        _ground_pose(description)
        if description.aabb is None:
            _ground_aabb(description)
        if description.obb is None:
            _ground_obb(description)
    _update_object_pose_in_knowrob(description.name, description.pose)
    yield description.__dict__


def call_ground(desig):
    type_to_function = {LocatedObjectDesignatorDescription: ground_located_object}
    ground_function = type_to_function[type(desig._description)]
    return ground_function(desig._description)


ObjectDesignator.resolvers['grounding'] = call_ground
