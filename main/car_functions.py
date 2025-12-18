"""
This code is made for move car.
Expected to use in main.py
"""

import numpy as np
import pybullet as p


def isContact(car_id, track_id, runoff_id, wheel_link_id):
    """
    Args:
        car_id (int) : Expected car id
        field_id (int) : Expected tarck id
        runoff_id (int) : Expected runoff id
        wheel_link_id (list) : each wheels id were stored list

    Return:
        flag (list) : True or False were stored list

    """

    wheel_contact = []
    for wheel in wheel_link_id:
        contacts = p.getContactPoints(
            bodyA=car_id,
            linkIndexA=wheel
        )

        surface = "air"

        for c in contacts:
            bodyB = c[2]

            if bodyB == track_id:
                surface = "track"
            elif bodyB == runoff_id:
                surface = "runoff"
           
        wheel_contact.append(surface)
    
    return wheel_contact


def getDistance(car_id):
    ray_origitn_local = [1.2, 0.0, 0.3]
    ray_length = 10.0

    ray_dir_local = [1, 0, 0]

    pos, orn = p.getBasePositionAndOrientation(car_id)
    rot_mat = p.getMatrixFromQuaternion(orn)
    rot_mat = np.array(rot_mat).reshape(3, 3)
    
    # origin_world = pos + rot_mat @ np.array(ray_origin_local)