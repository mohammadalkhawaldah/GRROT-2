"""Geometry of MediaPipe model landmarks; never a calibrated robot pose."""
import numpy as np
from scipy.spatial.transform import Rotation

FINGERS = {'thumb': [0, 1, 2, 3, 4], 'index': [0, 5, 6, 7, 8],
           'middle': [0, 9, 10, 11, 12], 'ring': [0, 13, 14, 15, 16],
           'little': [0, 17, 18, 19, 20]}
EDGES = [(a, b) for chain in FINGERS.values() for a, b in zip(chain, chain[1:])]


def palm_pose(points):
    """Columns of R: little->index MCP, wrist->middle MCP, their cross product.

    R maps this constructed palm frame into MediaPipe's model-output axes.
    Neither the model origin nor the axes are a calibrated camera/world frame.
    """
    p = np.asarray(points, dtype=float)
    if p.shape != (21, 3) or not np.all(np.isfinite(p)):
        raise ValueError('Expected 21 finite XYZ model landmarks')
    longitudinal = p[9] - p[0]
    across = p[5] - p[17]
    length, width = np.linalg.norm(longitudinal), np.linalg.norm(across)
    if min(length, width) < 1e-6:
        raise ValueError('Degenerate palm length/width')
    y = longitudinal / length
    x = across - np.dot(across, y) * y
    sine = np.linalg.norm(x) / width
    if sine < .15:
        raise ValueError('Palm axes nearly collinear')
    x /= np.linalg.norm(x)
    z = np.cross(x, y)
    z /= np.linalg.norm(z)
    x = np.cross(y, z)
    matrix = np.column_stack([x, y, z])
    center = p[[0, 5, 9, 13, 17]].mean(axis=0)
    return center, matrix, Rotation.from_matrix(matrix).as_quat(), float(sine)


def bend(a, b, c):
    """Unsigned geometric bend (0 straight), not an anatomical joint rotation."""
    u, v = np.asarray(a) - b, np.asarray(c) - b
    denominator = np.linalg.norm(u) * np.linalg.norm(v)
    if denominator < 1e-10:
        return float('nan')
    return float(180 - np.degrees(np.arccos(np.clip(np.dot(u, v) / denominator, -1, 1))))


def finger_bends(points):
    angles = {}
    for name, chain in FINGERS.items():
        labels = ['cmc_proxy', 'mcp', 'ip'] if name == 'thumb' else ['mcp_proxy', 'pip', 'dip']
        for k, label in enumerate(labels, 1):
            a, b, c = (points[i] for i in chain[k-1:k+2])
            angles[f'{name}_{label}_bend_deg'] = bend(a, b, c)
    return angles


def angular_distance(q1, q2):
    return float(np.degrees((Rotation.from_quat(q1).inv() * Rotation.from_quat(q2)).magnitude()))
