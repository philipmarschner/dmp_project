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



robot_ip = "192.168.1.111"
fs = 500


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


current_pose = rtde_r.getActualTCPPose()

print(current_pose)