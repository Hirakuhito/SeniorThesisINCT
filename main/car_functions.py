"""
This code is made for move car.
Expected to use in main.py
"""

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