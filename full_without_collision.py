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
import momentum_observer.admittance as admittance
from momentum_observer.livefilter import LiveLFilter
import scipy
import csv
import time
import argparse
import sys
from  momentum_observer.observer import Observer as obs
from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy


robot_ip = "192.168.1.111"
fs = 500
#"192.168.1.111"

#sys.path.append('/home/jacob/workspace/dmp_project/dmp_pp/dmp')

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


UR5e = rtb.models.DH.UR5e()
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

observer_c = obs(20,UR5e)

Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])



admittance_c = admittance.Admitance(UR5e,0,0,Tc,100)
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300


rtde_c.endTeachMode()


plotting = True

if plotting:
    #file = open('FT_data_log6.csv', mode='w', newline='')
    #writer = csv.writer(file)
    rtde_r.startFileRecording("log_of_final_demonstration.csv")

#concatenate data


i = 0

# Parameters
demo_filename = "finaltest.csv"
demo = pd.read_csv(demo_filename, delimiter=",")


p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()


t_steps = len(p)
dt = 1.0/500.0  # 2ms


tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 300


MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
original_target = MP.imitate_path(x_des=p)


init_pose = rtde_r.getActualTCPPose()
init_position = p[0,:]

init_pose[0] = init_position[0]
init_pose[1] = init_position[1]
init_pose[2] = init_position[2]


#init_pose = rtde_r.getActualTCPPose()

rtde_c.moveL(init_pose, velocity, acceleration)
time.sleep(1)



p_out = np.zeros(p.shape)


current_pose = copy.deepcopy(init_pose)


fs = 500

b, a = scipy.signal.iirfilter(5, Wn=5, fs=fs, btype="high", ftype="butter")

live_lfilterr1 = LiveLFilter(b, a)
live_lfilterr2 = LiveLFilter(b, a)
live_lfilterr3 = LiveLFilter(b, a)
live_lfilterr4 = LiveLFilter(b, a)
live_lfilterr5 = LiveLFilter(b, a)
live_lfilterr6 = LiveLFilter(b, a)

start_currents = rtde_r.getActualCurrent()

currents_buffer = [start_currents,start_currents,start_currents]

downscaler = 0

iterations = 0

torques = np.zeros((3,6))

r_buffer = np.zeros((3,6)).tolist()

torque_constant = np.array([0.098322,0.098322,0.098322,0.07695,0.07695,0.07695])

filteredr1 = np.zeros((3,6))
filteredr2 = np.zeros((3,6))
filteredr3 = np.zeros((3,6))
filteredr4 = np.zeros((3,6))
filteredr5 = np.zeros((3,6))
filteredr6 = np.zeros((3,6))

post_collision = False
record_retrain = False
retrained_poses = []
run_admittance = False
run_retrained = False

run_fillter = False

# define IO ports
White = 6
Green = 5
Black = 4
Red = 7

try:
    while True:
        t_start = rtde_c.initPeriod()

        if(i >= len(ts)-1):
            print("Ts is done")
            break
            
        # 1. Strean DMP to robot
        if not post_collision:
            p_temp, _, _ = MP.step()
            p_out[i,:] = p_temp
            i += 1

            
            current_position = p_temp[0:3].tolist()

            current_orientation = init_pose[3:6]

            current_pose[0] = current_position[0]
            current_pose[1] = current_position[1]
            current_pose[2] = current_position[2]

            if run_fillter:
                if downscaler % 3 == 0:
                
                    downscaler = 0
                    currents = rtde_r.getActualCurrent()
                    currents_buffer.insert(0,currents)
                    currents_buffer.pop()
                    iterations += 1
                    currents_array = np.array(currents_buffer)
                    torques = np.array(currents)*torque_constant
                    actual_q = rtde_r.getActualQ()
                    actual_qd = rtde_r.getActualQd()
    
                    r = observer_c.calcR(torques,dt,actual_q,actual_qd)
    
    
                    r_list = r.tolist()
                    r_buffer.insert(0,r_list)
                    r_buffer.pop()
    
                    r_array = np.array(r_buffer)
    
    
                    r1 = r_array[:,0]
                    r2 = r_array[:,1]
                    r3 = r_array[:,2]
                    r4 = r_array[:,3]
                    r5 = r_array[:,4]
                    r6 = r_array[:,5]
    
                    filteredr1 = scipy.signal.medfilt(np.abs([live_lfilterr1._process(y) for y in r1]),3)
                    filteredr2 = scipy.signal.medfilt(np.abs([live_lfilterr2._process(y) for y in r2]),3)
                    filteredr3 = scipy.signal.medfilt(np.abs([live_lfilterr3._process(y) for y in r3]),3)
                    filteredr4 = scipy.signal.medfilt(np.abs([live_lfilterr4._process(y) for y in r4]),3)
                    filteredr5 = scipy.signal.medfilt(np.abs([live_lfilterr5._process(y) for y in r5]),3)
                    filteredr6 = scipy.signal.medfilt(np.abs([live_lfilterr6._process(y) for y in r6]),3)
    
                    filteredsum = filteredr1 + filteredr2 + filteredr3 + filteredr4 + filteredr5 + filteredr6
                    #print(filteredsum[1])
    
    
                    if filteredsum[1] > 3 and iterations > 100:
                        print("Collision detected")
                        collision = True
    
                        print("is protective stopped : " ,rtde_r.isProtectiveStopped())
                        rtde_c.servoStop()
    
    
                        break

                
            downscaler += 1
            
            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)


        # 2. Catch collision and setup admittance controller
        if rtde_r.getDigitalInState(Black) and not post_collision:
            t0 = i*dt
            s0 = MP.cs.s
            rtde_c.servoStop()
            post_collision = True
            print("Robot is protective stopped")
            time.sleep(2)
            rtde_c.zeroFtSensor()
            time.sleep(2)
            print("Admittance controller is setup")
            run_admittance = True

        # 3. Start recording data for retraining
        if rtde_r.getDigitalInState(Green) and post_collision and not record_retrain:
            record_retrain = True
            print("Recording data for retraining")

        # 4. Record data for new dmp
        if post_collision and run_admittance:
            training = True

            wrench = rtde_r.getActualTCPForce()

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
            if record_retrain:
                s1 = MP.cs.s
                MP.step()
                t1 = (i-1)*dt
                i += 1
                retrained_poses.append(current_pose[0:3])

            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
           
        # 5. Stop recording data and retrain dmp
        if rtde_r.getDigitalInState(Red) and post_collision:
            rtde_c.servoStop()
            run_admittance = False
            record_retrain = False
            print("Stopped recording data for retraining")
            print("Number of recorded poses: ", len(retrained_poses))

            # save recorded poses to file
            np.savetxt("retrained_poses.txt", np.array(retrained_poses), delimiter=",")
            
            xnew = np.array(retrained_poses)
            new_target = MP.retrain(x_new = xnew, f_target_original = original_target, t0 = t0, t1 = t1, s0 = s0, s1 = s1)

            retrained_MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier',w = MP.w, x_0 = MP.x_0, x_goal = MP.x_goal)
            
            
            p_out_retrained = np.zeros(p.shape)
            run_retrained = True
            zeta = 0
            print("Retrained DMP is setup")
            time.sleep(1)
            print("3...")
            time.sleep(1)
            print("2...")
            time.sleep(1)
            print("1...")
            time.sleep(1)
            print("Running retrained DMP")
            waypoint1 = [-0.40618171,  0.01946607,  0.46231492, -2.8211937,  -1.36856942,  0.01423411]
            waypoint2 = [-6.20752571e-03, -7.13086081e-01,  3.93015192e-01, -7.46020711e-01, -3.05170720e+00, -1.53451753e-04]
            rtde_c.moveL(waypoint1, velocity, acceleration)
            rtde_c.moveL(waypoint2, velocity, acceleration)
            rtde_c.moveL(init_pose, velocity, acceleration)
            time.sleep(1)
            

        # 6. Run retrained DMP
        if run_retrained:
            if zeta <= len(ts):
                p_temp_retrained, _, _ = retrained_MP.step()
                p_out_retrained[zeta,:] = p_temp_retrained
                current_position = p_temp_retrained[0:3].tolist()

                current_orientation = init_pose[3:6]

                current_pose[0] = current_position[0]
                current_pose[1] = current_position[1]
                current_pose[2] = current_position[2]

                rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
                zeta += 1
            else:
                print("Finished running retrained DMP")
                exit()
            
    
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    if plotting:
        #file.close()
        rtde_r.stopFileRecording()

    rtde_c.servoStop()
    rtde_c.stopScript()
