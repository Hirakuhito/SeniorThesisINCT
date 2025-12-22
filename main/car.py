import numpy as np
import pybullet as p


class Car:
    def __init__(self, car_id):
        self.car_id = car_id
    
        # sensor config
        self.sensor_front = {
            "origin": np.array([0.0, 0.5, 0.0]),
            "direction": np.array([0.0, 1.0, -0.05]),
            "length": 5.0
        }

        self.sensor_right = {
            "origin": np.array([0.2, 0.0, 0.0]),
            "direction": np.array([0.5, 0.0, -0.1]),
            "length": 2.0
        }

        self.sensor_left = {
            "origin": np.array([-0.2, 0.0, 0.0]),
            "direction": np.array([-0.5, 0.0, -0.1]),
            "length": 2.0
        }

        self.sensor_back = {
            "origin": np.array([0.0, -0.3, 0.0]),
            "direction": np.array([0.0, -1.0, -0.05]),
            "length": 3.0
        }

        self.sensors = [
            self.sensor_front,
            self.sensor_left,
            self.sensor_right,
            self.sensor_back,
        ]

    def local2world(self, pos, orn, sensor):
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        origin_world = np.array(pos) + rot @ sensor["origin"]
        dir_world = rot @ sensor["direction"]
        end_world = origin_world + dir_world * sensor["length"]

        return origin_world.tolist(), end_world.tolist()
    
    def checkHit(self):
        pos, orn = p.getBasePositionAndOrientation(self.car_id)

        starts = []
        ends = []

        for vec in self.sensors:
            s, e = self.local2world(pos, orn, vec)
            starts.append(s)
            ends.append(e)
            
        for s, e in zip(starts, ends):
            p.addUserDebugLine(s, e, [1, 0, 0])

        result = p.rayTestBatch(starts, ends)

        hit_front = result[0][2] * self.sensor_front["length"]
        hit_left = result[1][2] * self.sensor_left["length"]
        hit_right = result[2][2] * self.sensor_right["length"]
        hit_back = result[3][2] * self.sensor_back["length"]

        hit = [hit_front, hit_left, hit_right, hit_back]

        return hit
    
    # def isContact(car_id, track_id, runoff_id, wheel_link_id):
    #     """
    #     Args:
    #         car_id (int) : Expected car id
    #         field_id (int) : Expected tarck id
    #         runoff_id (int) : Expected runoff id
    #         wheel_link_id (list) : each wheels id were stored list

    #     Return:
    #         flag (list) : True or False were stored list
    #     """

    #     wheel_contact = []
    #     for wheel in wheel_link_id:
    #         contacts = p.getContactPoints(
    #             bodyA=car_id,
    #             linkIndexA=wheel
    #         )

    #         surface = "air"

    #         for c in contacts:
    #             bodyB = c[2]

    #             if bodyB == track_id:
    #                 surface = "track"
    #             elif bodyB == runoff_id:
    #                 surface = "runoff"
            
    #         wheel_contact.append(surface)
        
    #     return wheel_contact