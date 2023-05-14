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
UR5e = rtb.models.DH.UR5e()
#UR5eplot = rtb.models.UR5e()
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

observer_c = observer.Observer(10,UR5e)
admittance_c = admittance.Admitance(UR5e,0,0,Tc,120)
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300

#wait for 2 seconds
init_pose = rtde_r.getActualTCPPose()



rtde_c.moveL(init_pose, vel, acc)


#TODO: rotate wrenn to tool frame. Look at picture taken last time.

time.sleep(2)
rtde_c.zeroFtSensor()
time.sleep(2)

rtde_c.endTeachMode()





#rtde_c.moveJ([4.45956563949585, -1.7731076679625453, -1.3856301307678223, -1.5122645658305665, 1.5766737461090088, -2.2725346724139612], 0.5, 0.3)


print("done")

plotting = True

if plotting:
    #file = open('FT_data_log6.csv', mode='w', newline='')
    #writer = csv.writer(file)
    rtde_r.startFileRecording("Observer_Test_500hz.csv")

#concatenate data


i = 0

#[4.45956563949585, -1.7731076679625453, -1.3856301307678223, -1.5122645658305665, 1.5766737461090088, -2.2725346724139612]
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

        current_pose = rtde_r.getActualTCPPose()

        

        current_position = current_pose[0:3]
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


            

            


        rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
    
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    if plotting:
        #file.close()
        rtde_r.stopFileRecording()

    rtde_c.servoStop()
    rtde_c.stopScript()
