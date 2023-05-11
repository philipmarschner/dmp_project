import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg
import pandas

import rtde_control
import rtde_receive


import matplotlib.pyplot as plt
import time
from scipy.fft import fft, ifft, fftfreq

from scipy import signal

from scipy.fft import fftshift
from numpy import genfromtxt
from livefilter import LiveLFilter
#my_data = genfromtxt('../FT_data_log6.csv', delimiter=',')
demo = pandas.read_csv('/home/jacob/workspace/dmp_project/momentum_observer/dmp_collision_500hz_slaps.csv', delimiter=',')

q = demo[['actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']].to_numpy()
qd = demo[['actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']].to_numpy()
p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
aa = demo[['actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()
currents = demo[['actual_current_0', 'actual_current_1', 'actual_current_2', 'actual_current_3', 'actual_current_4', 'actual_current_5']].to_numpy()
target_moments = demo[['target_moment_0', 'target_moment_1', 'target_moment_2', 'target_moment_3', 'target_moment_4', 'target_moment_5']].to_numpy()
target_currents = demo[['target_current_0', 'target_current_1', 'target_current_2', 'target_current_3', 'target_current_4', 'target_current_5']].to_numpy()
pose = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()

#rtde_c = rtde_control.RTDEControlInterface("192.168.1.111")
#rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.111")

#rtde_control.zeroFtSensor()


import observer

dt = 0.002


#UR5e = rtb.models.DH.UR5e()
#UR5plot = rtb.models.UR5()
UR5e = rtb.models.DH.UR5e()
#UR5eplot = rtb.models.UR5e()

observer = observer.Observer(20,UR5e)

#torque constant for first three joints 

torque_constant = np.array([0.098322,0.098322,0.098322,0.07695,0.07695,0.07695])
#torque_constant = np.array([0.07695,0.07695,0.07695,0.07695,0.07695,0.07695])
#torques = currents*torque_constant
torques = currents*torque_constant

fs = 500

b, a = signal.iirfilter(5, Wn=10, fs=fs, btype="high", ftype="butter")

live_lfilterr1 = LiveLFilter(b, a)
live_lfilterr2 = LiveLFilter(b, a)
live_lfilterr3 = LiveLFilter(b, a)
live_lfilterr4 = LiveLFilter(b, a)
live_lfilterr5 = LiveLFilter(b, a)
live_lfilterr6 = LiveLFilter(b, a)

import time
st = time.time()

r = [observer.calcR(torques[i,:],dt,q[i,:],qd[i,:]) for i in range(0,q.shape[0],5)]
r_array = np.array(r)


r1 = r_array[:,0]
r2 = r_array[:,1]
r3 = r_array[:,2]
r4 = r_array[:,3]
r5 = r_array[:,4]
r6 = r_array[:,5]


## simulate live filter - passing values one by one
filteredr1 = signal.medfilt(np.abs([live_lfilterr1._process(y) for y in r1]),3)
filteredr2 = signal.medfilt(np.abs([live_lfilterr2._process(y) for y in r2]),3)
filteredr3 = signal.medfilt(np.abs([live_lfilterr3._process(y) for y in r3]),3)
filteredr4 = signal.medfilt(np.abs([live_lfilterr4._process(y) for y in r4]),3)
filteredr5 = signal.medfilt(np.abs([live_lfilterr5._process(y) for y in r5]),3)
filteredr6 = signal.medfilt(np.abs([live_lfilterr6._process(y) for y in r6]),3)
et = time.time()
#signal.medfilt(np.abs([live_lfilterr1._process(y) for y in r1]),3)
#signal.medfilt(np.abs([live_lfilterr2._process(y) for y in r2]),3)
#signal.medfilt(np.abs([live_lfilterr3._process(y) for y in r3]),3)
#signal.medfilt(np.abs([live_lfilterr4._process(y) for y in r4]),3)
#signal.medfilt(np.abs([live_lfilterr5._process(y) for y in r5]),3)
#signal.medfilt(np.abs([live_lfilterr6._process(y) for y in r6]),3)


#time.sleep(1)
time_taken = et-st
#print('Execution time:', time_taken, 'miliseconds')


#fig, axs = plt.subplots(6, 2)
#
##plt.plot(signal.medfilt(np.abs(filteredx[100:]),5))
#
axs[0, 0].plot(r1)
axs[0, 0].set_title('R1')
axs[0, 1].plot(filteredr1[100:])
axs[0, 1].set_title('Filtered R1')
axs[1, 0].plot(r2)
axs[1, 0].set_title('R2')
axs[1, 1].plot(filteredr2[100:])
axs[1, 1].set_title('Filtered R2')
axs[2, 0].plot(r3)
axs[2, 0].set_title('R3')
axs[2, 1].plot(filteredr3[100:])
axs[2, 1].set_title('Filtered R3')
axs[3, 0].plot(r4)
axs[3, 0].set_title('R4')
axs[3, 1].plot(filteredr4[100:])
axs[3, 1].set_title('Filtered R4')
axs[4, 0].plot(r5)
axs[4, 0].set_title('R5')
axs[4, 1].plot(filteredr5[100:])
axs[4, 1].set_title('Filtered R5')
axs[5, 0].plot(r6)
axs[5, 0].set_title('R6')
axs[5, 1].plot(filteredr6[100:])
axs[5, 1].set_title('Filtered R6')
#
#
#norm1 = (filteredr1-np.min(filteredr1))/(np.max(filteredr1)-np.min(filteredr1))
#norm2 = (filteredr2-np.min(filteredr2))/(np.max(filteredr2)-np.min(filteredr2))
#norm3 = (filteredr3-np.min(filteredr3))/(np.max(filteredr3)-np.min(filteredr3))
#norm4 = (filteredr4-np.min(filteredr4))/(np.max(filteredr4)-np.min(filteredr4))
#norm5 = (filteredr5-np.min(filteredr5))/(np.max(filteredr5)-np.min(filteredr5))
#norm6 = (filteredr6-np.min(filteredr6))/(np.max(filteredr6)-np.min(filteredr6))



#observer.calcR(torques[0,:].reshape(6,1),dt)

#plt.subplot(6,3,1)
#plt.plot(r_array[:,0])
#plt.subplot(6,3,2)
#plt.plot(r_array[:,1])
#plt.subplot(6,3,3)
#plt.plot(r_array[:,2])
#plt.subplot(6,3,4)
#plt.plot(r_array[:,3])
#plt.subplot(6,3,5)
#plt.plot(r_array[:,4])
#plt.subplot(6,3,6)
#plt.plot(r_array[:,5])
#
#plt.subplot(6,3,7)
#plt.plot(q[:,0])
#plt.subplot(6,3,8)
#plt.plot(q[:,1])
#plt.subplot(6,3,9)
#plt.plot(q[:,2])
#plt.subplot(6,3,10)
#plt.plot(q[:,3])
#plt.subplot(6,3,11)
#plt.plot(q[:,4])
#plt.subplot(6,3,12)
#
#plt.plot(q[:,5])
#
#plt.subplot(6,3,13)
#plt.plot(qd[:,0])
#plt.subplot(6,3,14)
#plt.plot(qd[:,1])
#plt.subplot(6,3,15)
#plt.plot(qd[:,2])
#plt.subplot(6,3,16)
#plt.plot(qd[:,3])
#plt.subplot(6,3,17)
#plt.plot(qd[:,4])
#plt.subplot(6,3,18)
#plt.plot(qd[:,5])
#
plt.show()





#read csv by collum name

#UR5e.q = my_data







#env = swift.Swift()
#env.launch(realtime=True)
#env.add(UR5eplot)
t = 0





#for i in range(1000):
#    qdd = UR5.accel(UR5.q, tau, UR5.qd)
#    env.step(0.05)
#
#print(observer.model.q)

#recorded_r = np.zeros([6,1])

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
#import matplotlib.pyplot as plt
#plt.plot(recorded_r[0,:])
#plt.plot(recorded_r[1,:])
#plt.plot(recorded_r[2,:])
#plt.plot(recorded_r[3,:])
#plt.plot(recorded_r[4,:])
#plt.plot(recorded_r[5,:])
#
#plt.show()




