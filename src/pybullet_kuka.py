import pybullet as p
import numpy as np
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


p.loadURDF("plane.urdf")
p.loadURDF("kuka_iiwa/model.urdf",useFixedBase=1)

while (1):
    p.stepSimulation()
    time.sleep(0.001)