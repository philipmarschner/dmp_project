import roboticstoolbox as rtb
import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import momentum_observer.admittance as admittance
from momentum_observer.livefilter import LiveLFilter
import scipy
import time
from  momentum_observer.observer import Observer as obs
from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import pandas as pd
import copy

#-------------------------------------------------------
# Parameters
#-------------------------------------------------------

robot_ip = "192.168.1.111"
rtde_frequency = 400.0
fs = rtde_frequency
dt = 1.0/rtde_frequency

vel = 0.5
acc = 0.5

iterations = 0
i = 0

# define IO ports
White = 6
Green = 5
Black = 4
Red = 7


#-------------------------------------------------------
# High pass filter setup
#-------------------------------------------------------
b, a = scipy.signal.iirfilter(5, Wn=20, fs=fs, btype="high", ftype="butter")
live_lfilterr1 = LiveLFilter(b, a)
live_lfilterr2 = LiveLFilter(b, a)
live_lfilterr3 = LiveLFilter(b, a)
live_lfilterr4 = LiveLFilter(b, a)
live_lfilterr5 = LiveLFilter(b, a)
live_lfilterr6 = LiveLFilter(b, a)
run_fillter = True

#-------------------------------------------------------
# RTDE setup
#-------------------------------------------------------
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002

velocity = 0.5
acceleration = 0.5
lookahead_time = 0.1
gain = 150
rt_receive_priority = 90
rt_control_priority = 85
rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
rtde_c.endTeachMode()

#-------------------------------------------------------
# GMO SETUP
#-------------------------------------------------------
#model = rtb.models.DH.UR5e()
model = None
observer_c = obs(30,model=model)
torque_constant = np.array([0.098322,0.098322,0.098322,0.07695,0.07695,0.07695])


#-------------------------------------------------------
# Admittance controller setup
#-------------------------------------------------------
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
admittance_c = admittance.Admitance(robot=model,kp=0,ko=0,Tc=Tc,kdp=100,dt=dt)


#-------------------------------------------------------
# Setup data logging
#-------------------------------------------------------
plotting = True

if plotting:
    rtde_r.startFileRecording("log_of_final_demonstration_test_observer1.csv")

retrained_poses = []

#-------------------------------------------------------
# Load demonstration
#-------------------------------------------------------
demo_filename = "./momentum_observer/Observer_Test_500hz.csv"
demo = pd.read_csv(demo_filename, delimiter=",")
p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
t_steps = len(p)

#-------------------------------------------------------
# Setup parameters for DMP
#-------------------------------------------------------
tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 300

#-------------------------------------------------------
# Setup and train DMP
#-------------------------------------------------------
MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
original_target = MP.imitate_path(x_des=p)

#-------------------------------------------------------
# Move robot to start position
#-------------------------------------------------------

# Generate initial pose with current orientation
init_pose = rtde_r.getActualTCPPose()
init_position = p[0,:]
init_pose[0:3] = init_position[0:3]

rtde_c.moveL(init_pose, velocity, acceleration)
time.sleep(1)

current_pose = copy.deepcopy(init_pose)


#-------------------------------------------------------
# Setup state machine
#-------------------------------------------------------
# DEFINE STATES
STATE = -1
STATE_DONE = 0
STATE_STREAM_DMP = 1
STATE_COLLISION_DETECTED = 2
STATE_ADMITTANCE_CONTROL = 3
STATE_RECORD = 4
STATE_RETRAIN_DMP = 5
STATE_STREAM_RETRAINED_DMP = 6

# Set initial state
STATE = STATE_STREAM_DMP


#-------------------------------------------------------
# Main loop
#-------------------------------------------------------
try:
    while not STATE == STATE_DONE:
        test_start = time.time()
        t_start = rtde_c.initPeriod()


        #-------------------------------------------------------
        # Check for state changes
        #-------------------------------------------------------

        # Switch state if DMP is stream successfully
        if(i >= len(ts)-1):
            #print("Ts is done")
            STATE = STATE_DONE
            break
        
        # Switch state if start recording button is pressed
        if rtde_r.getDigitalInState(Green) and STATE == STATE_ADMITTANCE_CONTROL:
            print("Recording data for retraining")
        
        # Switch state if manual collision detection is triggered
        if rtde_r.getDigitalInState(Black) and STATE == STATE_STREAM_DMP:
            STATE = STATE_COLLISION_DETECTED
            print("Collision triggered by button")

        # Switch state if stop recording button is pressed
        if rtde_r.getDigitalInState(Red) and STATE == STATE_RECORD:
            STATE = STATE_RETRAIN_DMP
        

        #-------------------------------------------------------
        # STATE MACHINE
        #-------------------------------------------------------
        
        # 1. Strean DMP to robot
        if STATE == STATE_STREAM_DMP:
            dmp_time = time.time()
            p_temp, _, _ = MP.step()
 
            dmp_end = time.time()

            #if iterations % 50 == 0:
            #    print("DMP time: ", dmp_end-dmp_time)

            i += 1

            
            current_position = p_temp[0:3].tolist()

            current_pose[0:3] = current_position[0:3]


            if run_fillter:
                if True:
                    test_start_filter = time.time()
                    currents = rtde_r.getActualCurrent()
                    iterations += 1
                    torques = np.array(currents)*torque_constant
                    actual_q = rtde_r.getActualQ()
                    actual_qd = rtde_r.getActualQd()
    
                    r = observer_c.calcR(torques,dt,actual_q,actual_qd)

                    filteredr1 = live_lfilterr1._process(r[0])
                    filteredr2 = live_lfilterr2._process(r[1])
                    filteredr3 = live_lfilterr3._process(r[2])
                    filteredr4 = live_lfilterr4._process(r[3])
                    filteredr5 = live_lfilterr5._process(r[4])
                    filteredr6 = live_lfilterr6._process(r[5])

                    filteredr = np.array([filteredr1,filteredr2,filteredr3,filteredr4,filteredr5,filteredr6])
    
                    filteredsum = np.abs(filteredr1) + np.abs(filteredr2) + np.abs(filteredr3) + np.abs(filteredr4) + np.abs(filteredr5) + np.abs(filteredr6)
                    test_end_filter = time.time()

                    #if iterations % 50 == 0:
                    #    print("Time for filter: ", test_end_filter - test_start_filter)
                    #    print("Frequency of filter: ", 1/(test_end_filter - test_start_filter))

                    if iterations % 50 == 0:
                        print("Filtered sum: ", filteredsum)

                        
    
                    if filteredsum > 0.5 and iterations > 1000:
                       print("Collision detected")
                       collision = True
                       STATE = STATE_COLLISION_DETECTED
    
                       print("is protective stopped : " ,rtde_r.isProtectiveStopped())
                       rtde_c.servoStop()
                       time.sleep(2)

                       exit()
                       
                       break
                       #continue
                    #    #Continue to next iteration
            
            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)


        # 2. Catch collision and setup admittance controller
        if STATE == STATE_COLLISION_DETECTED:
            t0 = i*dt
            s0 = MP.cs.s
            rtde_c.servoStop()
            print("Robot is protective stopped")
            time.sleep(2)
            rtde_c.zeroFtSensor()
            time.sleep(2)
            print("Admittance controller is setup")
            STATE = STATE_ADMITTANCE_CONTROL

        # 4. Admittance controller
        if STATE == STATE_ADMITTANCE_CONTROL or STATE == STATE_RECORD:

            wrench = rtde_r.getActualTCPForce()

            deltapos, deltaori = admittance_c.step(wrench)

            current_pose = rtde_r.getActualTCPPose()

            

            current_position = current_pose[0:3]
            #convert current_position to numpy array
            current_position = np.array(current_position)

            new_position = current_position + deltapos.T

            #replace first 3 elements of current_pose with new_position
            new_position = new_position.tolist()[0]
            current_pose[0:3] = new_position[0:3]
            
            if STATE == STATE_RECORD:
                s1 = MP.cs.s
                MP.step()
                t1 = (i-1)*dt
                i += 1
                retrained_poses.append(current_pose[0:3])

            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)

        # 5. Stop recording data and retrain dmp
        if STATE == STATE_RETRAIN_DMP:
            rtde_c.servoStop()
            print("Stopped recording data for retraining")
            print("Number of recorded poses: ", len(retrained_poses))

            # save recorded poses to file
            np.savetxt("retrained_poses_test_observer.csv", np.array(retrained_poses), delimiter=",")
            
            xnew = np.array(retrained_poses)
            new_target = MP.retrain(x_new = xnew, f_target_original = original_target, t0 = t0, t1 = t1, s0 = s0, s1 = s1)

            retrained_MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier',w = MP.w, x_0 = MP.x_0, x_goal = MP.x_goal)
            
            
            p_out_retrained = np.zeros(p.shape)
            i_new_dmp = 0
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
            STATE = STATE_STREAM_RETRAINED_DMP
            

        # 6. Run retrained DMP
        if STATE == STATE_STREAM_RETRAINED_DMP:
            if i_new_dmp <= len(ts):
                p_temp_retrained, _, _ = retrained_MP.step()
                p_out_retrained[i_new_dmp,:] = p_temp_retrained
                current_position = p_temp_retrained[0:3].tolist()

                current_pose[0:3] = current_position[0:3]

                rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
                i_new_dmp += 1
            else:
                STATE = STATE_DONE
                print("Finished running retrained DMP")
            
        test_end = time.time()
        rtde_c.waitPeriod(t_start)
        #if iterations%10 == 0:
        #    print("Time for loop: ", test_end - test_start)
        #    print("Frequency of loop: ", 1/(test_end - test_start))
        #    print("\n")

except KeyboardInterrupt:
    if plotting:
        #file.close()
        rtde_r.stopFileRecording()

    rtde_c.servoStop()
    rtde_c.stopScript()
