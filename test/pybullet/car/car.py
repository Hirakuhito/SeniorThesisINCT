import time

import pybullet as p
import pybullet_data as pd

#*-------- variables ---------------
model_data_path = "../../../data/"
use_realtime_sim = 0

#*-------- basic settings ----------
engine_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

p.resetSimulation()
p.setGravity(0, 0, -9.8)

#*-------- load model -------------
stadium = p.loadSDF("stadium.sdf")
car = p.loadURDF("{}racecar/racecar.urdf".format(model_data_path))

#*-------- additional settigns -----
# show Joint info of car
for i in range(p.getNumJoints(car)):
    print(p.getJointInfo(car, i))

# Disable non-drive wheels 
inactive_wheels = [5, 7]
active_wheels = [2, 3]
steering = [4, 6]

for wheel in inactive_wheels:
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

# add slider (velocity, Max torque, steering angle)
target_velocity_slider = p.addUserDebugParameter("wheel velocity", -10, 10, 0)
max_torque_slider = p.addUserDebugParameter("max torque", 0, 10, 0)
steering_angle_slider = p.addUserDebugParameter("steering angle", -0.5, 0.5, 0)

#*--------- simulation ------------
while (True):
    # get slider's value
    target_velocity = p.readUserDebugParameter(target_velocity_slider)
    max_torque = p.readUserDebugParameter(max_torque_slider)
    steering_angle = p.readUserDebugParameter(steering_angle_slider)

    # drive wheel's speed control
    for wheel in active_wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=max_torque)

    # steering wheel's position control
    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steering_angle)

    if (use_realtime_sim == 0):
        p.stepSimulation()

    time.sleep(1. / 240.)        