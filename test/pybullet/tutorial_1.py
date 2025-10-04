import time

import pybullet as p
import pybullet_data as pd

#=========== basic setting ==============
engine_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)

time_step = 1. / 240.

#========== load objects ===============
startPos = [0, 0, 3]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

planeId = p.laodURDF("plane.urdf")
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# set the center of mass frame
p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# start simulation
for i in range (1000):
    p.stepSimulation()
    time.sleep(time_step)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)
p.disconnect()