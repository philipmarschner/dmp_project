import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import csv


robot_ip = "192.168.1.111"
#"192.168.1.111"


vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002


lookahead_time = 0.1
gain = 600
rt_receive_priority = 90
rt_control_priority = 85
rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)


# Create csv file with 12 columns
# Open the CSV file in append mode and create a CSV writer object
file = open('FT_data_z_slaps.csv', mode='w', newline='')
writer = csv.writer(file)
#header = ['fx','fy','fz','tau_x','tau_y','tau_z','posx','posy','posz','rot1','rot2','rot3']
#writer.writerow(header)

time.sleep(2)
rtde_c.zeroFtSensor()
time.sleep(2) # wait for sensor to zero


notStopped = True
print('recording')
try:
    while notStopped:
        t_start = rtde_c.initPeriod()
        wrench = rtde_r.getActualTCPForce()


        current_pose = rtde_r.getActualTCPPose()
        row = wrench + current_pose
        writer.writerow(row)
        #sleep for 0.1 seconds
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
        print("Control Interrupted!")
        file.close()
        rtde_c.servoStop()
        rtde_c.stopScript()

