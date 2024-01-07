import numpy as np
from scipy.spatial.transform import Rotation as R
from sdf_converter.common_types import Link, Pose

def compute_relative_transformation(rot_parent, d_parent, rot_child, d_child):
    """
    Computes the relative transformation for the child frame expressed in parent frame.

    :param rot_parent: The rotation matrix of the parent frame expressed in the world frame.
    :param d_parent: The translation vector of the parent frame expressed in the world frame.
    :param rot_child: The rotation matrix of the child frame expressed in the world frame.
    :param d_child: The translation vector of the child frame expressed in the world frame.

    :return: The relative transformation of the child frame express in the parent frame.
             The translation part is a 3d vector and the rotation is also a 3d vector in rpy format.
    """

    H_p_inv = np.zeros((4,4))
    H_p_inv[:3, :3] = rot_parent.T
    H_p_inv[:3, -1] = -np.matmul(rot_parent.T, d_parent)
    H_p_inv[3, 3] = 1

    H_c = np.zeros((4, 4))
    H_c[:3, :3] = rot_child
    H_c[:3, -1] = d_child
    H_c[-1, -1] = 1

    H_relative = np.matmul(H_p_inv, H_c)

    translation = H_relative[:3, -1]
    rotation_mat = H_relative[:3, :3]
    rotation = R.from_matrix(rotation_mat).as_euler("zyx", degrees=False)
    return translation, rotation


def compute_relative_pose(parent_link: Link, child_link: Link):
    parent_pose = parent_link.pose if parent_link.pose is not None else Pose.default()
    child_pose = child_link.pose if child_link.pose is not None else Pose.default()
    rot_parent = R.from_euler("zyx", parent_pose.get_rpy(), degrees=False).as_matrix()
    d_parent = parent_pose.get_translation()
    rot_child = R.from_euler("zyx", child_pose.get_rpy(), degrees=False).as_matrix()
    d_child = child_pose.get_translation()
    translation, rotation = compute_relative_transformation(rot_parent, d_parent, rot_child, d_child)
    return Pose.from_vec(translation, rotation)
