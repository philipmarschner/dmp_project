import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg
import time
import rtde_control
import rtde_receive
import admittance

ip = "192.168.1.111"
#"192.168.1.111"
rtde_c = rtde_control.RTDEControlInterface(ip)
rtde_r = rtde_receive.RTDEReceiveInterface(ip)



import observer

dt = 0.02


#UR5e = rtb.models.DH.UR5e()
#UR5plot = rtb.models.UR5()
UR5e = rtb.models.DH.UR5e()
#UR5eplot = rtb.models.UR5e()
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

observer_c = observer.Observer(10,UR5e)
admittance_c = admittance.Admitance(UR5e,0,0,Tc)
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300

print(rtde_r.getFtRawWrench())

rtde_c.zeroFtSensor()

print(rtde_r.getFtRawWrench())

rtde_c.stopScript()


#
#while True:
#
#    try:
#        t_start = rtde_c.initPeriod()
#        wrench = rtde_r.getFtRawWrench()
#        deltapos, deltaori = admittance_c.step(wrench)
#
#        print("wrench: ",wrench)
#        print("deltapos: ",deltapos)
#
#        rtde_c.waitPeriod(t_start)
#
#    except KeyboardInterrupt:
#        print("Control Interrupted!")
#        #rtde_c.servoStop()
#        rtde_c.stopScript()
#
