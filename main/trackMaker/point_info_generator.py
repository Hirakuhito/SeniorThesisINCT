"""
This program is designed exclusively for Oval circuit.
"""
import numpy as np


def gen_center_point(length, radius, segments=100, pos=np.array([0, 0])):
    """
    Args:
        length (float) : straight length of circuit.
        radius (float) : corner radius of circuit.
        segments (int) : Division number of each section. (straight -> corner -> straight)
        pos (np.array) : center of circuit
        
    Return:
        points (list) : center line points.
    """

    if length < 0 or radius < 0 or segments < 0:
        raise ValueError("'length', 'radius' and 'segments' must be positive number.")
    
    if isinstance(segments, int):
        raise TypeError("Segments must be integer.")

    #*=============== vailables ====================
    #* All points are stored list.
    all_points = []

    #* offset
    corner_offset = length / 2 #* center of cerner section [0, coner_offset]

    #*============== sections ======================
    #* Upper straight
    y_straight = np.linspace(corner_offset, -corner_offset, segments, endpoint=True)
    x_straight = np.full_like(y_straight, radius)
    upper_straight = np.stack((x_straight, y_straight), axis=1)

    all_points.append(upper_straight)