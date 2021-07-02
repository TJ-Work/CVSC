import numpy as np
from tf import transformations

import numpy as np

def yaw_from_pose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw


def dist_of_poses(pose0, pose1):
    point_0 = np.array([
        pose0.position.x,
        pose0.position.y,
        pose0.position.z 
    ])
    point_1 = np.array([
        pose1.position.x,
        pose1.position.y,
        pose1.position.z 
    ])
    diff = point_0 - point_1
    return np.sqrt(np.sum(np.square(diff)))

def curvature_from_poses(pose0, pose1):
    yaw_0 = yaw_from_pose(pose0)
    yaw_1 = yaw_from_pose(pose1)
    distance = dist_of_poses(pose0, pose1)

    return (yaw_1 - yaw_0) / distance
