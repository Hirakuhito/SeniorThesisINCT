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
    ray_origin_local_front = [0.0, 0.5, 0.0]
    ray_origin_local_back = [0.0, -0.3, 0.0]
    # ray_dir_local = [0, 1, -0.1]
    ray_length = 3
    ray_num = 11
    fov = np.deg2rad(120)

    pos, orn = p.getBasePositionAndOrientation(car_id)
    yaw = p.getEulerFromQuaternion(orn)[2]

    cyf, syf = np.cos(yaw), np.sin(yaw)
    rot_zf = np.array([
        [cyf, -syf, 0],
        [syf,  cyf, 0],
        [ 0,   0, 1]
    ])

    #todo I wanna make back ray sensor but I have no idea to do how...

    origin_world_front = pos + rot_zf @ np.array(ray_origin_local_front)

    angles_f = np.linspace(-fov/2, fov/2, ray_num)

    hit_info = []
    for a in angles_f:
        ca, sa = np.cos(a), np.sin(a)
        dir_local = np.array([sa, ca, -0.03])

        dir_world    = rot_zf @ np.array(dir_local)
        end_world    = origin_world_front + dir_world * ray_length

        # p.addUserDebugLine(origin_world, end_world, [1,0,0], 1, 0.1)
        results_f = p.rayTest(origin_world_front.tolist(), end_world.tolist())[0]


        isHit = results_f[0] != -1
        hit_id = results_f[0]

        hit_info.append([isHit, hit_id])

    match show:
        case True:
            if dict == None:
                raise ValueError("If you wanna show the information, you should attach dictionary.")

            match isHit:
                case True:
                    print(f"# Hit to {dict.get(hit_id)}".ljust(30), end="\r")

                case False:
                    print(f"# Not hit to anything.".ljust(30), end="\r")

        case False:
            pass

    return hit_info
