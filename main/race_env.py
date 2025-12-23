import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data as pd
from gymnasium import spaces
from stable_baselines3.common.env_checker import check_env

import trackMaker.track_info_generator as pg
from car import Car


class RacingEnv(gym.Env):
    def __init__(self, gui=False):
        super().__init__()

        self.engine_id = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())
        p.setTimeStep(1. / 60.)
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

        self._build_world()
        self.car = Car(self.car_id)

        self.start_pos = [self.radius, 0, 0.2]
        self.start_orn = p.getQuaternionFromEuler([0, 0, 0])


    def _build_world(self):
        self.straight = 7
        self.radius = 3
        self.width = 2

        points = pg.gen_center_point(self.straight, self.radius)

        track_mesh_points, _ = pg.gen_mesh_data(points, self.width, self.radius, in_out="in")
        pg.export_obj(track_mesh_points, "track", in_out="in")

        runoff_mesh_points, _ = pg.gen_mesh_data(points, self.width, self.radius, in_out="out")
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
        car_base_pos = [self.radius, 0, 0.2]
        car_base_orient = p.getQuaternionFromEuler([0, 0, 0])

        self.car_id = p.loadURDF(car_path, basePosition=car_base_pos, baseOrientation=car_base_orient, globalScaling=0.2)


    def _calc_reward(self):
        pos, orn = p.getBasePositionAndOrientation(self.car_id)

        rot = p.getMatrixFromQuaternion(orn)
        forward = np.array([rot[0], rot[3], rot[6]])

        vel, _ = p.getBaseVelocity(self.car_id)
        vel = np.array(vel)

        forward_speed = np.dot(forward, vel)

        reward = forward_speed * 0.5

        return reward


    def _get_obs(self):
        hit_data = self.car.checkHit()

        obs = []
        for sensor in hit_data:
            obs.extend(sensor)

        return np.array(obs, dtype=np.float32)
    

    def _is_terminated(self):
        pos, _ = p.getBasePositionAndOrientation(self.car_id)

        if pos[2] < 0.1:
            return True
        
        return False

    def reset(self, *,  seed=None, options=None):
        super().reset(seed=seed)

        p.resetBasePositionAndOrientation(
            self.car_id,
            self.start_pos,
            self.start_orn
        )
        p.resetBaseVelocity(self.car_id, [0, 0, 0], [0, 0, 0])

        self.car.reset()

        obs = self._get_obs()
        info = {}

        return obs, info
    
    def step(self, action):
        steer, throttle = action
        self.car.apply_action(steer, throttle)

        for _ in range(2):
            p.stepSimulation()

        obs = self._get_obs()
        reward = self._calc_reward()

        terminated = self._is_terminated()
        truncated = False

        return obs, reward, terminated, truncated, {}
        

env = RacingEnv(gui=True)
obs, _ = env.reset()

check_env(env, warn=True)

for _ in range(100):
    action = env.action_space.sample()
    obs, reward, done, trunc, _ = env.step(action)