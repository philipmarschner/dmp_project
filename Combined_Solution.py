from matplotlib import pyplot as plt
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import datetime
import math
import sys
import os
import psutil
import numpy as np
import time
import scipy
import roboticstoolbox as rtb
from scipy import signal
from momentum_observer import observer

from momentum_observer.admittance import Admitance  
#import momentum_observer.livefilter as filter
from momentum_observer.livefilter import LiveLFilter
from momentum_observer.observer import Observer


# ------------  Defines  -------------------
#
# ------------------------------------------


robot_ip = "192.168.1.111"
fs = 500
vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
lookahead_time = 0.1
gain = 300

median_n_sample = 3

# Filter 
b, a = scipy.signal.iirfilter(4, Wn=10, fs=fs, btype="low", ftype="butter")
live_lfilterfx = LiveLFilter(b, a)
live_lfilterfy = LiveLFilter(b, a)
live_lfilterfz = LiveLFilter(b, a)


flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002

rt_receive_priority = 90
rt_control_priority = 85
rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)




robot = rtb.models.DH.UR5e()

Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
O_ko = 20

observer_c = Observer(O_ko,robot)
admittance_c = Admitance(robot,0,0,Tc,70)

init_pose = rtde_r.getActualTCPPose()


# ------------ Module -------------------
#
#          calibrate FTSensor 
#
#  --------------------------------------

def cali_FTSensor(wait_before_zeroFTSensor = 2, wait_after_zeroFTSensor = 2):
    rtde_c.moveL(init_pose, vel, acc)
    time.sleep(wait_before_zeroFTSensor)
    rtde_c.zeroFtSensor()
    time.sleep(wait_after_zeroFTSensor)
    rtde_c.endTeachMode()



# ------------ Module -------------------
#
#           Record trajectory 
#
#  --------------------------------------

def record_dmp():
    plotting = True

    if plotting:
        print("starting recording")
        rtde_r.startFileRecording("log_admittance_control3.csv")

    i = 0

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
                if i % 10 == 0:
                    sys.stdout.write("\r")
                    sys.stdout.write("{:3d} samples.".format(i))
                    sys.stdout.flush()

                i += 1

            rtde_c.servoL(current_pose, vel, acc, dt, lookahead_time, gain)
        
            rtde_c.waitPeriod(t_start)

    except KeyboardInterrupt:
        if plotting:
            rtde_r.stopFileRecording()

        rtde_c.servoStop()
        rtde_c.stopScript()
        print("Done recording")



# ------------ Module -------------------
#
#             Observer
#
#  --------------------------------------

def observer_calcR(torques, q, qd, i):
    r = observer_c.calcR(torques[i],dt, q[i],qd[i])
    r_array = np.array(r)
    return r_array 

def extend_r_array(r_arr, temp_arr):
    r_arr.extend(temp_arr)

def median_buffer_check(r_arr, n_sample):
    if r_arr.shape().m >= n_sample:
        return True
    else: 
        return False


def Observer_with_filter(r_arr, i, n_sample):
    r1 = r_arr[i-n_sample:i,0]
    r2 = r_arr[i-n_sample:i,1]
    r3 = r_arr[i-n_sample:i,2]
    r4 = r_arr[i-n_sample:i,3]
    r5 = r_arr[i-n_sample:i,4]
    r6 = r_arr[i-n_sample:i,5]
    
    ## simulate live filter - passing values one by one
    filteredr1 = signal.medfilt(np.abs([live_lfilterfx._process(y) for y in r1]),n_sample)
    filteredr2 = signal.medfilt(np.abs([live_lfilterfy._process(y) for y in r2]),n_sample)
    filteredr3 = signal.medfilt(np.abs([live_lfilterfz._process(y) for y in r3]),n_sample)
    filteredr4 = signal.medfilt(np.abs([live_lfilterfz._process(y) for y in r4]),n_sample)
    filteredr5 = signal.medfilt(np.abs([live_lfilterfz._process(y) for y in r5]),n_sample)
    filteredr6 = signal.medfilt(np.abs([live_lfilterfz._process(y) for y in r6]),n_sample)

    #filtered = [filteredr1, filteredr2, filteredr3, filteredr4, filteredr5, filteredr6]

    norm1 = (filteredr1-np.min(filteredr1))/(np.max(filteredr1)-np.min(filteredr1))
    norm2 = (filteredr2-np.min(filteredr2))/(np.max(filteredr2)-np.min(filteredr2))
    norm3 = (filteredr3-np.min(filteredr3))/(np.max(filteredr3)-np.min(filteredr3))
    norm4 = (filteredr4-np.min(filteredr4))/(np.max(filteredr4)-np.min(filteredr4))
    norm5 = (filteredr5-np.min(filteredr5))/(np.max(filteredr5)-np.min(filteredr5))
    norm6 = (filteredr6-np.min(filteredr6))/(np.max(filteredr6)-np.min(filteredr6))

    norm = [norm1, norm2, norm3, norm4, norm5, norm6]
    return norm

# ------------ main loop -------------------
i = 0
while True:
    # record dmp
    cali_FTSensor()
    record_dmp()

    # admittance


    # observer
    r_arr = [0]
    
    torques = 0
    q = 0
    qd = 0
    r_arr = extend_r_array(observer_calcR(torques, q, qd, i=i))
    if median_buffer_check(r_arr, median_n_sample):
        Observer_with_filter(r_arr, i, median_n_sample)

