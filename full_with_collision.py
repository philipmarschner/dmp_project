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
b, a = scipy.signal.iirfilter(5, Wn=5, fs=fs, btype="high", ftype="butter")
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
observer_c = obs(50)
torque_constant = np.array([0.098322,0.098322,0.098322,0.07695,0.07695,0.07695])*101


#-------------------------------------------------------
# Admittance controller setup
#-------------------------------------------------------
Tc = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
admittance_teach = admittance.Admitance(kp=0,ko=0,Tc=Tc,kdp=100,kdo = 120,dt=dt)
admittance_run = admittance.Admitance(kp=200,ko=0,Tc=Tc,kdp=150,kdo = 120,dt=dt)


#-------------------------------------------------------
# Setup data logging
#-------------------------------------------------------
plotting = True

if plotting:
    rtde_r.startFileRecording("Demonstraiton_of_trajectory_May_17_log1.csv")

retrained_poses = []

#-------------------------------------------------------
# Load demonstration
#-------------------------------------------------------
demo_filename = "./momentum_observer/Demonstraiton_of_trajectory_May_17_from_side_guesstimation.csv"
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
# Move robot to start posimitate_pathition
#-------------------------------------------------------

# Generate initial pose with current orientation
init_pose = rtde_r.getActualTCPPose()
init_position = p[0,:]
init_pose[0:3] = init_position[0:3]

rtde_c.moveL(init_pose, velocity, acceleration)
time.sleep(1)

current_pose = copy.deepcopy(init_pose)
pose_buffer = []
phase_buffer = []
i_buffer = []
x_buffer = []
dx_buffer = []
ddx_buffer = []
for _ in range(0, 100):
    pose_buffer.append(current_pose)
    phase_buffer.append(MP.cs.s)
    i_buffer.append(i)
    x_buffer.append(MP.x)
    dx_buffer.append(MP.dx)
    ddx_buffer.append(MP.ddx)

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
PRINT_FILTER = False
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
        if(i >= len(ts)-1) and STATE == STATE_STREAM_DMP:
            #print("Ts is done")
            STATE = STATE_DONE
            break

        # Switch state if new DMP is stream successfully
        if(i >= len(ts)-1) and STATE == STATE_STREAM_DMP:
                STATE = STATE_DONE
                print("Finished running retrained DMP")
        
        # Switch state if manual collision detection is triggered
        if rtde_r.getDigitalInState(Black) and STATE == STATE_STREAM_DMP:
            STATE = STATE_COLLISION_DETECTED
            print("Collision triggered by button")

        # Switch state if start recording button is pressed
        if rtde_r.getDigitalInState(Green) and STATE == STATE_ADMITTANCE_CONTROL:
            print("Recording data for retraining")
            STATE = STATE_RECORD

        

        # Switch state if stop recording button is pressed
        if rtde_r.getDigitalInState(Red) and STATE == STATE_RECORD:
            print("Stop recording data")
            STATE = STATE_RETRAIN_DMP
        
        #-------------------------------------------------------
        # Check protective stop
        #-------------------------------------------------------
        if rtde_r.isProtectiveStopped() or rtde_r.isEmergencyStopped():
            PRINT_FILTER = True
        #    print("Protective stop or emergency stop triggered")
        #    print("##Filtered sum: ", filteredsum)
        #    print("##Filtered values: ", filteredr)
        
        #-------------------------------------------------------
        # Check collision
        #-------------------------------------------------------
        if run_fillter and STATE == STATE_STREAM_DMP:
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
                #filteredr6 = live_lfilterr6._process(r[5])

                filteredr = np.array([filteredr1,filteredr2,filteredr3,filteredr4,filteredr5]) #filteredr6])

                #filteredsum = np.abs(filteredr1) + np.abs(filteredr2) + np.abs(filteredr3) + np.abs(filteredr4) + np.abs(filteredr5) # + np.abs(filteredr6)

                filteredsum = np.abs(filteredr1) + np.abs(filteredr5)
                test_end_filter = time.time()

                #if iterations % 50 == 0:
                #    print("Time for filter: ", test_end_filter - test_start_filter)
                #    print("Frequency of filter: ", 1/(test_end_filter - test_start_filter))

                # if iterations % 100 == 0 or PRINT_FILTER:
                #     print("Filtered sum: ", filteredsum)
                #     print("Filtered values: ", filteredr)

                # if PRINT_FILTER:
                #     print("Filter values: ", filteredr)

                if filteredsum > 4 and iterations > 200:
                    print("Is protective stopped : " ,rtde_r.isProtectiveStopped())
                    # Move robot out of collision
                    #rtde_c.servoL(pose_buffer[0], velocity, acceleration, dt, lookahead_time, gain)
                    print("Collision detected, filtered sum: ", filteredsum)
                    collision = True
                    collision_pose = rtde_r.getActualTCPPose()
                    admittance_run.reset()
                    STATE = STATE_COLLISION_DETECTED

                    
                    #admittance_run.kp = np.eye(3)*0
                    #admittance_run.kdp = np.eye(3)*150
                    

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
            
            dmp_position = p_temp[0:3].tolist()

            
            current_pose[0:3] = dmp_position[0:3]

            # Add to buffers
            pose_buffer.pop(0)
            pose_buffer.append(copy.deepcopy(current_pose))
            phase_buffer.pop(0)
            phase_buffer.append(copy.deepcopy(MP.cs.s))
            i_buffer.pop(0)
            i_buffer.append(copy.deepcopy(i))
            x_buffer.pop(0)
            x_buffer.append(copy.deepcopy(MP.x))
            dx_buffer.pop(0)
            dx_buffer.append(copy.deepcopy(MP.dx))
            ddx_buffer.pop(0)
            ddx_buffer.append(copy.deepcopy(MP.ddx))

            # Add admittance controller
            wrench = rtde_r.getActualTCPForce()
            deltapos, deltaori = admittance_run.step(wrench)
            
            current_pose[0:3] = (np.array(current_pose[0:3])+deltapos.T).tolist()[0]
            # if i % 100 == 0:
            #     print(deltapos)
            # Stream calculated pose from dmp + admittance controller 
            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)





        # 2. Catch collision and setup admittance controller
        if STATE == STATE_COLLISION_DETECTED:
            i = i_buffer[0]
            t0 = i*dt
            s0 = phase_buffer[0]
            current_pose[0:3] = pose_buffer[0]
            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)
            #rtde_r.stopFileRecording()
            
            print("Zeroing FT sensor...")
            time.sleep(1)
            rtde_c.zeroFtSensor()
            time.sleep(1)
            print("Admittance controller is setup")
            STATE = STATE_ADMITTANCE_CONTROL
            
            #Restore DMP to buffered state
            MP.cs.s = phase_buffer[0]
            MP.x = x_buffer[0]
            MP.dx = dx_buffer[0]
            MP.ddx = ddx_buffer[0]

        # 4. Admittance controller
        if STATE == STATE_ADMITTANCE_CONTROL or STATE == STATE_RECORD:
            
            # Step DMP if in retrain state
            if STATE == STATE_RECORD:
                s1 = MP.cs.s
                p_temp, _, _ = MP.step()
                t1 = (i-1)*dt
                i += 1
                retrained_poses.append(current_pose[0:3])
                current_pose[0:3] = p_temp[0:3].tolist()[0]


            wrench = rtde_r.getActualTCPForce()
            deltapos, deltaori = admittance_teach.step(wrench)

        
            current_position = current_pose[0:3]
            #convert current_position to numpy array
            current_position = np.array(current_position)

            new_position = current_position + deltapos.T

            #replace first 3 elements of current_pose with new_position
            new_position = new_position.tolist()[0]
            current_pose[0:3] = new_position[0:3]

            rtde_c.servoL(current_pose, velocity, acceleration, dt, lookahead_time, gain)



        # 5. Stop recording data and retrain dmp
        if STATE == STATE_RETRAIN_DMP:
            rtde_c.servoStop()
            print("Stopped recording data for retraining")
            print("Number of recorded poses: ", len(retrained_poses))

            # save recorded poses to file
            np.savetxt("retrained_poses_test_observer.csv", np.array(retrained_poses), delimiter=",")
            
            # Prepare for retraining
            xnew = np.array(retrained_poses)
            new_target = MP.retrain(x_new = xnew, f_target_original = original_target, t0 = t0, t1 = t1, s0 = s0, s1 = s1)

            retrained_MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier',w = MP.w, x_0 = MP.x_0, x_goal = MP.x_goal)
            
            
            p_out_retrained = np.zeros(p.shape)
            i = 0
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
            
            
        test_end = time.time()
        rtde_c.waitPeriod(t_start)


except KeyboardInterrupt:
    if plotting:
        #file.close()
        rtde_r.stopFileRecording()

    rtde_c.servoStop()
    rtde_c.stopScript()
