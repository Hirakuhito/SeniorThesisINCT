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


def checkHit(car_id, show=False, dict=None):
    ray_origin_local = [1.2, 0.0, 0.3]
    ray_length = 10

    ray_dir_local = [1, 0, -0.5]

    pos, orn = p.getBasePositionAndOrientation(car_id)
    rot_mat = p.getMatrixFromQuaternion(orn)
    rot_mat = np.array(rot_mat).reshape(3, 3)
    
    origin_world = pos + rot_mat @ np.array(ray_origin_local)
    dir_world = rot_mat @ np.array(ray_dir_local)
    end_world = origin_world + dir_world * ray_length

    hit_result = p.rayTest(origin_world.tolist(), end_world.tolist())[0]

    isHit = hit_result[0] != -1
    hit_id = hit_result[0]
    hit_frac = hit_result[2]
    distance = hit_frac * ray_length

    hit_info = [isHit, hit_id, distance]

    match show:
        case True:
            #* check dict -> search name from id -> print
            pass

        case False:
            pass

    return hit_info
