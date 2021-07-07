from collections import defaultdict

import numpy as np
import pybullet as pb
from am_control_plugin_python.data.common_data import Extent, PoseQuaternion
from am_control_plugin_python.rps_interface.http_interface import RPSInterface
from pytransform3d.rotations import quaternion_wxyz_from_xyzw
from pytransform3d.transformations import transform_from_pq, pq_from_transform
from rospkg import RosPack

from pycram.bullet_world import BulletWorld, Object
from pycram.pycram_industrial.rps_helpers import assert_ok


class RPSSceneMirror(object):
    def __init__(self, rps: RPSInterface, world: BulletWorld):
        rospack = RosPack()
        self.remote_package_locations = {
            rospack.get_path("iai_maps"): "C:/Users/alt/devel/catkin_ws/src/iai_maps/iai_maps",
            rospack.get_path("iai_kitchen"): "C:/Users/alt/devel/catkin_ws/src/iai_maps/iai_kitchen"
        }
        self.link_idx_to_rps_object_id = {}  # Mapping body ID --> link ID --> RPS ID
        self.rps = rps
        self.world = world

    def setup_scene(self):
        for obj in self.world.objects:
            self._add_object_to_scene(obj)

    def update_scene(self):
        # Add all objects if they do not exist
        for obj in self.world.objects:
            if obj.id not in self.link_idx_to_rps_object_id.keys():
                self._add_object_to_scene(obj)
        for obj in self.world.objects:
            self._update_object_pose(obj)

    def clear_scene(self):
        res = assert_ok(self.rps.get_object_info())
        for obj in res["objects"]:
            self._remove_object_from_scene(obj["objectID"])

    def _replace_package_paths(self, path: str) -> str:
        for local_package_path in self.remote_package_locations.keys():
            if local_package_path in path:
                return path.replace(local_package_path, self.remote_package_locations[local_package_path])
        return path

    def _add_object_to_scene(self, obj: Object):
        if obj.type == "robot":
            return
        self.link_idx_to_rps_object_id[obj.id] = {}

        num_joints = pb.getNumJoints(obj.id)

        # Link names
        link_names = []
        for joint_idx in range(num_joints):
            joint_info = pb.getJointInfo(obj.id, joint_idx)
            link_names.append(joint_info[12].decode("utf-8"))

        # Link poses
        link_states = pb.getLinkStates(obj.id, list(range(num_joints)))
        for link_idx, link_state in enumerate(link_states):

            world_link_pos = link_state[4]
            world_link_ori = link_state[5]
            world_link_frame = transform_from_pq(
                np.concatenate((world_link_pos, quaternion_wxyz_from_xyzw(world_link_ori))))

            collision_shape_data = pb.getCollisionShapeData(obj.id, link_idx)
            if len(collision_shape_data) == 0:  # No collision object
                continue
            extents = collision_shape_data[0][3]
            extents_mm = Extent(extents[0] * 1000, extents[1] * 1000, extents[2] * 1000)
            collision_shape = collision_shape_data[0][2]

            obj_pose = pq_from_transform(world_link_frame)
            obj_pose = PoseQuaternion(*(obj_pose[:3] * 1000).tolist(), *(obj_pose[3:]).tolist())
            if collision_shape == pb.GEOM_BOX:
                res = assert_ok(self.rps.insert_object("box", obj_pose, name=link_names[link_idx], extents=extents_mm))
            elif collision_shape == pb.GEOM_MESH:
                mesh_filepath = self._replace_package_paths(collision_shape_data[0][4].decode("utf-8"))
                print(f"Importing {mesh_filepath} into RPS")
                res = assert_ok(self.rps.insert_object(mesh_filepath, obj_pose, name=link_names[link_idx]))
            else:
                raise RuntimeError(f"Unknown collision shape type {collision_shape}")
            self.link_idx_to_rps_object_id[obj.id][link_idx] = res["objectID"]

    def _remove_object_from_scene(self, object_id: str):
        assert_ok(self.rps.remove_model(object_id))

    def _update_object_pose(self, obj: Object):
        if obj.type == "robot":
            return

        num_joints = pb.getNumJoints(obj.id)

        # Get current link poses from world and update in RPS
        link_states = pb.getLinkStates(obj.id, list(range(num_joints)))
        for link_idx, link_state in enumerate(link_states):
            # Skip if link has no collision model --> these are not in the RPS
            collision_shape_data = pb.getCollisionShapeData(obj.id, link_idx)
            if len(collision_shape_data) == 0:
                continue

            world_link_pos = link_state[4]
            world_link_ori = link_state[5]
            world_link_frame = transform_from_pq(
                np.concatenate((world_link_pos, quaternion_wxyz_from_xyzw(world_link_ori))))
            obj_pose = pq_from_transform(world_link_frame)
            obj_pose = PoseQuaternion(*(obj_pose[:3] * 1000).tolist(), *(obj_pose[3:]).tolist())

            assert_ok(self.rps.update_model(self.link_idx_to_rps_object_id[obj.id][link_idx], obj_pose))
