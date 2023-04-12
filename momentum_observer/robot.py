
import observer as obs
import roboticstoolbox as rtb

import numpy as np
import spatialmath as sm




robot = rtb.models.DH.UR5()

observer = obs.Observer(0.1,robot)

robot.plot(q=[0,0,0,0,0,0])

#wait for key press
input("Press Enter to continue...")
