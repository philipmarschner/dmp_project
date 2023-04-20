import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg

import rtde_control
import rtde_receive

rtde_c = rtde_control.RTDEControlInterface("192.168.1.111")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.111")

rtde_control.zeroFtSensor()


import observer

dt = 0.02


#UR5e = rtb.models.DH.UR5e()
#UR5plot = rtb.models.UR5()
UR5e = rtb.models.DH.UR5e()
#UR5eplot = rtb.models.UR5e()

observer = observer.Observer(10,UR5e)


#env = swift.Swift()
#env.launch(realtime=True)
#env.add(UR5eplot)
t = 0





#for i in range(1000):
#    qdd = UR5.accel(UR5.q, tau, UR5.qd)
#    env.step(0.05)
#
#print(observer.model.q)

recorded_r = np.zeros([6,1])

#while t < 10:
#        
#    tau = np.array([1,1,1,1,1,1])
#
#    if t < 5:
#        tau_observer = tau
#    elif t >= 5 and t < 7:
#        tau_observer = np.array([1.5,1.5,1.5,1.5,1.5,1.5])
#    else:
#        tau_observer = tau
#
#
#    #qdd = np.linalg.inv(UR5e.inertia(UR5e.q))@(tau - UR5e.coriolis(UR5e.q,UR5e.qd)@UR5e.qd - UR5e.gravload(UR5e.q))
#    UR5e.qdd = qdd
#    UR5e.qd = UR5e.qd + qdd*dt
#    UR5e.q = UR5e.q + UR5e.qd*dt
#    UR5e.accel(UR5e.q,tau,UR5e.qd)
#
#    #env.step(dt)
#    t += dt
#    r = observer.calcR(tau_observer,dt)
#
#    recorded_r = np.concatenate([recorded_r,r.reshape(6,1)],axis=1)




#plot recorded r
import matplotlib.pyplot as plt
plt.plot(recorded_r[0,:])
plt.plot(recorded_r[1,:])
plt.plot(recorded_r[2,:])
plt.plot(recorded_r[3,:])
plt.plot(recorded_r[4,:])
plt.plot(recorded_r[5,:])

plt.show()




