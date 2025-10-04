import time

import pybullet as p
import pybullet_data as pd

#----- basic setting --------
engine_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

p.setGravity(0, 0, -9.8)

#------ model setting ----------
p.loadURDF("plane.urdf")

cube_start_pos = [0, 0, 3]
cube_start_ort = p.getQuaternionFromEuler([0, 0, 0])
box_id = p.loadURDF("r2d2.urdf", cube_start_pos, cube_start_ort)

cube_pos, cube_ort = p.getBasePositionAndOrientation(box_id)

#------- simulate --------------
useRealTimeSimulation = 0           # Disable real time simlation mode
if (useRealTimeSimulation):
    p.setTealTimeSimulation(1)

time_step = 1. / 240.
while 1:
    if (useRealTimeSimulation):
        p.setGravity(0, 0, -9.8)
        time.sleep(time_step)

    else:
        p.stepSimulation()