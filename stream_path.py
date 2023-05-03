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


    # Move to init position using moveL
    init_pose = getPose(pos[0], rot[0])
    rtde_c.moveL(init_pose, vel, acc)

    #wait for 5 seconds
    #time.sleep(2)


    i = 1
    try:
        while i < len(pos):
            t_start = rtde_c.initPeriod()
            servo_target = getPose(pos[i], rot[i])
            rtde_c.servoL(servo_target, vel, acc, dt, lookahead_time, gain)
            rtde_c.waitPeriod(t_start)
            i += 1


    except KeyboardInterrupt:
        print("Control Interrupted!")
        rtde_c.servoStop()
        rtde_c.stopScript()

    print("Trajectory streamed successfully")