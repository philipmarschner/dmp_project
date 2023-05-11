import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import admittance
from livefilter import LiveLFilter
import scipy
import csv
import time
import argparse
import sys
import observer
from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



robot_ip = "192.168.1.111"
fs = 500
#"192.168.1.111"

b, a = scipy.signal.iirfilter(4, Wn=10, fs=fs, btype="low", ftype="butter")
live_lfilterfx = LiveLFilter(b, a)
live_lfilterfy = LiveLFilter(b, a)
live_lfilterfz = LiveLFilter(b, a)

vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002
#robot_ip = "172.17.0.2"

lookahead_time = 0.1
gain = 600
rt_receive_priority = 90
rt_control_priority = 85
rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)



dt = 0.02


#UR5e = rtb.models.DH.UR5e()
#UR5plot = rtb.models.UR5()
UR5e = rtb.models.DH.UR5e()
#UR5eplot = rtb.models.UR5e()
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

observer_c = observer.Observer(20,UR5e)
admittance_c = admittance.Admitance(UR5e,0,0,Tc,50)
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300



time.sleep(2)
rtde_c.zeroFtSensor()
time.sleep(2)

rtde_c.endTeachMode()

print("done")

plotting = True

if plotting:
    #file = open('FT_data_log6.csv', mode='w', newline='')
    #writer = csv.writer(file)
    rtde_r.startFileRecording("log_admittance_control_dmp_nochair.csv")

#concatenate data


i = 0

# Parameters
demo_filename = "../recorded_data.csv"
demo = pd.read_csv(demo_filename, delimiter=",")


p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()


t_steps = len(p)
dt = 1.0/500.0  # 2ms


tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 100


#MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
MP.imitate_path(x_des=p)

init_pose = p[0,:]

rtde_c.moveL(init_pose, velocity, acceleration)
time.sleep(1)

p_out = np.zeros(p.shape)

for i in range(len(ts)):
    p_temp, _, _ = MP.step()
    p_out[i,:] = p_temp



try:
    while True:
        t_start = rtde_c.initPeriod()
        wrench = rtde_r.getActualTCPForce()    
        deltapos, deltaori = admittance_c.step(wrench)

        current_pose = rtde_r.getActualTCPPose()

        

        current_position = init_pose[0:3]
        current_orientation = init_pose[3:6]
        #convert current_position to numpy array
        current_position = np.array(current_position)

        #print("current_position :" ,current_position)
        new_position = current_position + deltapos.T


        #replace first 3 elements of current_pose with new_position

        new_position = new_position.tolist()[0]

        current_pose[0] = new_position[0]
        current_pose[1] = new_position[1]
        current_pose[2] = new_position[2]
    
        new_orientation = current_orientation

      

        if plotting:
            #row = admittance_c.exponentialFilter(wrench)[0] + wrench
            
            sys.stdout.write("\r")
            sys.stdout.write("{:3d} samples.".format(i))
            sys.stdout.flush()

            i += 1

            

            


        rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
    
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    if plotting:
        #file.close()
        rtde_r.stopFileRecording()

    rtde_c.servoStop()
    rtde_c.stopScript()
