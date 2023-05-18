from admittance import Admitance
import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from spatialmath import Quaternion as quat

def refTrajectory(t):
    return np.transpose(np.array([0.3*np.sin(t/3), 0.3*np.sin(t/3)*np.cos(t/3), 0.1*np.sin(t)])).reshape(3,1)

robot = rtb.models.DH.UR5()
robot.q = [0, 0, 0, 0, 0, 0]

# create data array with robot.q as first element

refdata = np.zeros(3).reshape(3,1)
admdata = np.zeros(3).reshape(3,1)
deltadata = np.zeros(3).reshape(3,1)
odata = np.zeros(4).reshape(4,1)
wdata = np.zeros(3).reshape(3,1)

Tc = np.array([[1,0,0,0],[0,1,0,0.1],[0,0,1,0.2],[0,0,0,1]])
admittanceController = Admitance(robot, kdp=25, kdo=25,ko=150,kp=15,Tc = Tc)

tempread = admittanceController.kdp

wrench = np.array([[0], [0], [0], [0], [0], [0]])
testwrench = np.array([[0], [0], [0], [0], [0], [0]])
t = 0


for i in range(25000):
    t +=0.002

    if t > 5 and t < 10:
        wrench = testwrench
    elif t > 15 and t < 20: 
        wrench = np.array([[2], [3], [4], [1], [2], [3]])
        

    else:
        wrench = np.array([[0], [0], [0], [0], [0], [0]])

    admittanceController.step(wrench)
    #data = np.append(data, refTrajectory(t)+admittanceController.getDeltaqPos())
    refdata = np.concatenate([refdata, refTrajectory(t)],axis=1)
    admdata = np.concatenate([admdata, admittanceController.getDeltaqPos()],axis=1)
    deltadata = np.concatenate([deltadata, admittanceController.deltaqPos],axis=1)
    odata = np.concatenate([odata, admittanceController.getDeltaQuat()],axis=1)
    wdata = np.concatenate([wdata, admittanceController.getDeltaW()],axis=1)
    
# plot position

#plt.plot(admdata[0,0:1000])

fig, axs = plt.subplots(3)
fig.suptitle('Vertically stacked subplots')
#axs[0].plot(refdata[0,0:25000])
axs[0].plot(admdata[0,0:25000])
#axs[1].plot(refdata[1,0:25000])
axs[1].plot(admdata[1,0:25000])
#axs[2].plot(refdata[2,0:25000])
axs[2].plot(admdata[2,0:25000])





deltafig, deltaaxs = plt.subplots(3)
deltafig.suptitle('Vertically stacked subplots')
deltaaxs[0].plot(deltadata[0,0:25000])
deltaaxs[1].plot(deltadata[1,0:25000])
deltaaxs[2].plot(deltadata[2,0:25000])

# plot orientation

ofig, oaxs = plt.subplots(4)
ofig.suptitle('ortientation')
oaxs[0].plot(odata[0,0:25000])
oaxs[1].plot(odata[1,0:25000])
oaxs[2].plot(odata[2,0:25000])
oaxs[3].plot(odata[3,0:25000])

#plot angular velocity

wfig, waxs = plt.subplots(3)
wfig.suptitle('angular velocity')
waxs[0].plot(wdata[0,0:25000])
waxs[1].plot(wdata[1,0:25000])
waxs[2].plot(wdata[2,0:25000])





#wait for user to close plot
plt.show()

plt.plot(odata[1,0:25000])
plt.plot(odata[2,0:25000])
plt.plot(odata[3,0:25000])

plt.show()

norot = quat(1,0,0,0)
#finalrot = quat(odata[0,25000],odata[1,25000],odata[2,25000],odata[3,25000])

#finalrot.animate(start = norot)




#wait for key
#input("Press Enter to continue...")


