from typing import List

from pycram.bullet_world import BulletWorld
from pycram.knowrob.reasoning import object_type, instances_of, object_pose
from pycram.object_designator import LocatedObjectDesignatorDescription, ObjectDesignator


def _update_object_pose_in_bullet_world(object_name: str, object_pose: List):
    if BulletWorld.current_bullet_world is None:
        return
    bullet_object_names = [obj.name for obj in BulletWorld.current_bullet_world.objects]
    if object_name in bullet_object_names:
        obj = BulletWorld.current_bullet_world.get_objects_by_name(object_name)[0]
        obj.set_position_and_orientation(object_pose[:3], object_pose[3:7])


def ground_located_object(description: LocatedObjectDesignatorDescription):
    def _ground_pose(desc):
        desc.pose = object_pose(desc.name, desc.reference_frame, desc.timestamp)

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
            if BulletWorld.current_bullet_world is not None:
                _update_object_pose_in_bullet_world(description.name, description.pose)
            yield description.__dict__
        raise StopIteration()
    elif not description.pose:
        # Fetch the object pose and yield the grounded description
        _ground_pose(description)
    _update_object_pose_in_bullet_world(description.name, description.pose)
    yield description.__dict__


def call_ground(desig):
    type_to_function = {LocatedObjectDesignatorDescription: ground_located_object}
    ground_function = type_to_function[type(desig._description)]
    return ground_function(desig._description)


ObjectDesignator.resolvers['grounding'] = call_ground