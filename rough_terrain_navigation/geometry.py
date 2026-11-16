"""Pure geometry; quaternions are ordered x, y, z, w."""
import math
import numpy as np


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def multiply_quaternions(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def split_pose(position, quaternion):
    """Return world-aligned planar pose and exact 3D residual to physical root."""
    q = np.asarray(quaternion, dtype=float)
    rotation = quaternion_matrix(q)
    q = q / np.linalg.norm(q)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    planar_q = np.array([0., 0., math.sin(yaw/2), math.cos(yaw/2)])
    inverse_yaw = np.array([0., 0., -planar_q[2], planar_q[3]])
    residual_q = multiply_quaternions(inverse_yaw, q)
    return (np.array([position[0], position[1], 0.]), planar_q,
            np.array([0., 0., position[2]]), residual_q)
