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
import quaternion
import time
import argparse
import sys
import copy


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



# Check for real time kernel
#checkKernel()

# ur_rtde realtime priorities


    


import observer

dt = 0.02


#UR5e = rtb.models.DH.UR5e()
#UR5plot = rtb.models.UR5()
#UR5e = rtb.models.DH.UR5e()
#UR5eplot = rtb.models.UR5e()
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])


observer_c = observer.Observer(10)
admittance_c = admittance.Admitance(0,0,Tc,70,120)
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300

#wait for 2 seconds
init_pose = rtde_r.getActualTCPPose()


time.sleep(2)
rtde_c.zeroFtSensor()
time.sleep(2)

rtde_c.endTeachMode()






print("Done zeroing sensor")

plotting = True

if plotting:
    #file = open('FT_data_log6.csv', mode='w', newline='')
    #writer = csv.writer(file)
    rtde_r.startFileRecording("./Demonstraiton_of_trajectory_May_17_from_side_guesstimation.csv")


Use_orientation = False

i = 0

current_pose =copy.deepcopy(init_pose)

try:
    while True:
        t_start = rtde_c.initPeriod()
        wrench = rtde_r.getActualTCPForce()

        #wrench = admittance_c.filter_wrench(wrench)
        #wrench = [0.1,0,0,0,0,0]
        #force = np.array(wrench[0:3])

        #print("force: ")
        #print(np.linalg.norm(force))

        #find largest element in force
        #max_force = np.argmax(np.abs(force))

        #if max_force < 1:
        #    wrench = [0,0,0,0,0,0]
    
        deltapos, deltaori = admittance_c.step(wrench)

        # Get current pose and split into position and orientation
        #current_pose = rtde_r.getActualTCPPose()

        #current_pose = copy.deepcopy(init_pose)
        current_position = init_pose[0:3]
        current_orientation= init_pose[3:6]


        #convert current_position and orientation to numpy array
        current_position = np.array(current_position)
        
        #if Use_orientation:
        #    current_orientation = np.array(current_orientation)
        #    
        #    # Add the output of the admittance controller to the current position and orientation
        #    
        #    quats = quaternion.from_rotation_vector(current_orientation)
        #    new_orientation = quaternion.as_rotation_vector(quats*deltaori)
#
#
        #    # Ensure that the orientations are formatted properly in AA
        #    if np.dot(new_orientation,current_orientation) < 0:
        #        new_orientation *= -1
#
        #    # Ensure that the quaternions do not flip sign
#
        #    new_orientation = quaternion.from_rotation_vector(new_orientation)
#
        #    if np.dot(quats.vec, new_orientation.vec) < 0:
        #        new_orientation *= -1
#
        #    #Convert orientation to AA
        #    new_orientation = quaternion.as_rotation_vector(new_orientation)
        #    
        #    #replace first 3 elements of current_pose with new_position
        #else:            
        #    new_orientation = current_orientation.tolist()[0]


        new_orientation = current_orientation
        new_position = (current_position + deltapos.T).tolist()[0]
        #new_position = new_position.tolist()[0]


        current_pose[0:3] = new_position[0:3]
        current_pose[3:6] = new_orientation[0:3]        

        rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
    
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    if plotting:
        #file.close()
        rtde_r.stopFileRecording()

    rtde_c.servoStop()
    rtde_c.stopScript()
