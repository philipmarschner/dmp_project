from rtde_receive import RTDEReceiveInterface as RTDEReceive
import datetime
import math
import sys
import os
import psutil
import numpy as np
import time
import roboticstoolbox as rtb
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import swift


if __name__ == '__main__':
    # Parameters
    robot_ip = "172.17.0.2"
    rtde_frequency = 500.0
    dt = 1.0/rtde_frequency  # 2ms
    rt_receive_priority = 90

    rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)

    env = swift.Swift()
    env.launch(realtime=True)
    

    UR5 = rtb.models.URDF.UR5()
    env.add(UR5)

    UR5.q = [0, 0, 0, 0, 0, 0]
    env.step()

    while True:
        t_start = rtde_r.initPeriod()
        time.sleep(0.1)
        q = rtde_r.getActualQ()
        UR5.q = q
        env.step()
        rtde_r.waitPeriod(t_start)
        


