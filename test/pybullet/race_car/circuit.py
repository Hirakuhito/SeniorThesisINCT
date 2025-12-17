import pybullet as p
import pybullet_data as pd 
import numpy as np 

#------- basic settings ---------
engine_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.resetSimulation()

#------- additional settings ----------
ground = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[50, 50, 0.1]
    )

ground_vi = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[50, 50, 0.1]
        rgbaColor=[0.6, 0.8, 0.6, 1]
    )

p.createMultiBody(
        baseMass=0,
    )
