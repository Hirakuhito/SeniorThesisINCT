import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data as pd
from gymnasium import spaces

import trackMaker.track_info_generator as pg
from car import Car


class RacingEnv(gym.Env):
    def __init__(self, gui=False):
        super().__init__()

        if gui:
            self.engine_id = p.connect(p.GUI)
        else:
            self.engine_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pd.getDataPath())
        p.setTimeStep(1. / 240.)
        p.setGravity(0, 0, -9.81)

        obs_dim = 18
        self.observation_space = spaces.Box(
            low=0.0, 
            high=1.0, 
            shape=(obs_dim,), 
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        self.car_id = None


    def _build_world(self):
        straight = 7
        radius = 3
        width = 2

        points = pg.gen_center_point(straight, radius)

        track_mesh_points, _ = pg.gen_mesh_data(points, width, radius, in_out="in")
        pg.export_obj(track_mesh_points, "track", in_out="in")

        runoff_mesh_points, _ = pg.gen_mesh_data(points, width, radius, in_out="out")
        pg.export_obj(runoff_mesh_points, "runoff", in_out="out")

        #*===================== load models ===================
        track_name = "track"
        runoff_name = "runoff"
        #* load circuit
        track_file_path = "circuitData/" + track_name + ".obj"
        runoff_file_path = "circuitData/" + runoff_name + ".obj"

        base_pos = [0, 0, 0]
        base_orient = p.getQuaternionFromEuler([0, 0, 0])

        track_vis_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=track_file_path,
            meshScale=[1.0, 1.0, 1.0],
            rgbaColor=[0.5, 0.5, 0.5, 1],
            specularColor=[0, 0, 0]
        )

        track_coll_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=track_file_path,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            meshScale=[1.0, 1.0, 1.0]
        )
        
        track_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=track_coll_id,
            baseVisualShapeIndex=track_vis_id,
            basePosition=base_pos,
            baseOrientation=base_orient
        )

        runoff_vis_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=runoff_file_path,
            meshScale=[1.0, 1.0, 1.0],
            rgbaColor=[0.2, 0.45, 0.2, 1],
            specularColor=[0, 0, 0]
        )

        runoff_coll_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=runoff_file_path,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            meshScale=[1.0, 1.0, 1.0]
        )

        runoff_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=runoff_coll_id,
            baseVisualShapeIndex=runoff_vis_id,
            basePosition=base_pos,
            baseOrientation=base_orient
        )


        #* load car
        car_path = "./formular/formular_car/car.urdf"
        car_base_pos = [radius, 0, 0.2]
        car_base_orient = p.getQuaternionFromEuler([0, 0, 0])

        self.car_id = p.loadURDF(car_path, basePosition=car_base_pos, baseOrientation=car_base_orient, globalScaling=0.2)


    def _get_obs(self):
        hit_data = self.car.checkHit()

        obs = []
        for sensor in hit_data:
            obs.extend(sensor)

        return np.array(obs, dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        p.setGravity(0, 0, -9.81)

        self.car = Car(self.car_id)

        obs = self._get_obs()
        
        return obs, {}
    
    def step(self, action):
        steer, throttle = action
        for _ in range(5):
            p.stepSimulation()

        obs = self._get_obs()

        terminated = False
        truncated = False

        reward = 0.0

        return obs, reward, terminated, truncated, {}
        

env = RacingEnv(gui=True)
env._build_world()
obs, _ = env.reset()

for _ in range(100):
    action = env.action_space.sample()
    obs, reward, done, trunc, _ = env.step(action)