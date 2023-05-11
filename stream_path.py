#Code copied from realtime_control_example.py
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import datetime
import math
import sys
import os
import psutil
import numpy as np
import time
from scipy import signal


import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg
import pandas
from scipy.fft import fftshift
from numpy import genfromtxt
from livefilter import LiveLFilter
import observer


def getPose(pos, rot):
    pose = np.array([pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]])
    return pose

def checkKernel():
    # Set application real-time priority
    os_used = sys.platform
    process = psutil.Process(os.getpid())
    if os_used == "win32":  # Windows (either 32-bit or 64-bit)
        process.nice(psutil.REALTIME_PRIORITY_CLASS)
    elif os_used == "linux":  # linux
        rt_app_priority = 80
        param = os.sched_param(rt_app_priority)
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except OSError:
            print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
        else:
            print("Process real-time priority set to: %u" % rt_app_priority)

def stream_traj(pos, rot, robot_ip):

    # Parameters
    vel = 0.5
    acc = 0.5
    rtde_frequency = 500.0
    dt = 1.0/rtde_frequency  # 2ms
    flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
    ur_cap_port = 50002
    #robot_ip = "172.17.0.2"

    lookahead_time = 0.1
    gain = 600

    # Check for real time kernel
    #checkKernel()

    # ur_rtde realtime priorities
    rt_receive_priority = 90
    rt_control_priority = 85

    rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
    rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

    UR5e = rtb.models.DH.UR5e()

    observer = observer.Observer(20,UR5e)


    r = [observer.calcR(torques[i,:],dt,q[i,:],qd[i,:]) for i in range(torques.shape[0])]
    r_array = np.array(r)

    fs = 500

    b, a = signal.iirfilter(5, Wn=10, fs=fs, btype="high", ftype="butter")

    live_lfilterr1 = LiveLFilter(b, a)
    live_lfilterr2 = LiveLFilter(b, a)
    live_lfilterr3 = LiveLFilter(b, a)
    live_lfilterr4 = LiveLFilter(b, a)
    live_lfilterr5 = LiveLFilter(b, a)
    live_lfilterr6 = LiveLFilter(b, a)



    plotting = True

    if plotting:
    #file = open('FT_data_log6.csv', mode='w', newline='')
    #writer = csv.writer(file)
        rtde_r.startFileRecording("log_admittance_control_dmp_with_slap_from_top.csv")

    # Move to init position using moveL
    init_pose = getPose(pos[0], rot[0])
    rtde_c.moveL(init_pose, vel, acc)

    #wait for 5 seconds
    #time.sleep(2)
    torque_constant = np.array([0.098322,0.098322,0.098322,0.07695,0.07695,0.07695])

    r_buffer = np.zeros((3,6))

    i = 1
    try:
        while i < len(pos):
            t_start = rtde_c.initPeriod()
            servo_target = getPose(pos[i], rot[i])
            rtde_c.servoL(servo_target, vel, acc, dt, lookahead_time, gain)

            currents = rtde_r.getActualCurrent()
            torques = currents*torque_constant
            q = rtde_r.getActualQ()
            qd = rtde_r.getActualQd()
            r = observer.calcR(torques[i,:],dt,q[i,:],qd[i,:])
            r_array = np.array(r)

            r1 = r_array[:,0]
            r2 = r_array[:,1]
            r3 = r_array[:,2]
            r4 = r_array[:,3]
            r5 = r_array[:,4]
            r6 = r_array[:,5]    

            filteredr1 = live_lfilterr1._process(r1)
            filteredr2 = live_lfilterr2._process(r2)
            filteredr3 = live_lfilterr3._process(r3)
            filteredr4 = live_lfilterr4._process(r4)
            filteredr5 = live_lfilterr5._process(r5)
            filteredr6 = live_lfilterr6._process(r6)

            filteredr = np.array([filteredr1,filteredr2,filteredr3,filteredr4,filteredr5,filteredr6])    

            r_buffer = np.concatenate((r_buffer,filteredr))
            r_buffer = np.delete(r_buffer,0,0)

            medianr1 = signal.medfilt(np.abs(filteredr1),3)


            rtde_c.waitPeriod(t_start)
            i += 1


    except KeyboardInterrupt:
        print("Control Interrupted!")
        rtde_c.servoStop()
        rtde_c.stopScript()

        if plotting:
      
            rtde_r.stopFileRecording()

    if plotting:
      
            rtde_r.stopFileRecording()


    print("Trajectory streamed successfully")