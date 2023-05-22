#from admittance import Admitance
import numpy as np
import pandas as pd
#import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import quaternion
import copy



def refTrajectory(t):
    return np.array([0.3*np.sin(t/3), 0.3*np.sin(t/3)*np.cos(t/3), 0.1*np.sin(t)])

def refOrientation(t):
    return np.array([1,0,0,0])

# robot = rtb.models.DH.UR5()
# robot.q = [0, 0, 0, 0, 0, 0]

demo_file ="/home/jannich/Documents/SDU/DMP/project/dmp_project/momentum_observer/May_22_log4_plane_demonstration.csv"
demo = pd.read_csv(demo_file, delimiter=",")
force_file = "/home/jannich/Documents/SDU/DMP/project/dmp_project/momentum_observer/May_22_log_compliance_demonstration.csv"
force = pd.read_csv(force_file, delimiter=",")

p_pos1 = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
p_rot_aa = demo[['actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()
f_pos1 = force[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
f_rot_aa = force[['actual_TCP_pose_3', 'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()

offset = 5
start = 10000
stop = 3000
f_pos_copy = f_pos1[start:len(f_pos1),:]
timestamps1 = len(f_pos_copy)
f_pos = f_pos_copy[:len(f_pos_copy)-stop-offset,:]

time = len(f_pos)
p_pos = p_pos1[len(p_pos1)-timestamps1+offset:len(p_pos1)-stop,:]

p_rot = quaternion.from_rotation_vector(p_rot_aa)
f_rot = quaternion.from_rotation_vector(f_rot_aa)



p_force =demo[['actual_TCP_force_0','actual_TCP_force_1','actual_TCP_force_2','actual_TCP_force_3','actual_TCP_force_4','actual_TCP_force_5']].to_numpy()
f_force =force[['actual_TCP_force_0','actual_TCP_force_1','actual_TCP_force_2','actual_TCP_force_3','actual_TCP_force_4','actual_TCP_force_5']].to_numpy()


f_force_copy = f_force[start:len(f_force),:]
timestam = len(f_force_copy)
f_force2 = f_force_copy[:len(f_force_copy)-stop-offset,:]

time = len(f_pos)
p_force2 = p_force[len(p_force)-timestam+offset:len(p_force)-stop,:]

# Tc = np.array([[1,0,0,0],[0,1,0,0.1],[0,0,1,0.2],[0,0,0,1]])
# admittanceController = Admitance(robot = robot, kp = 100, ko = 25,Tc = Tc, kdp = 100)

# tempread = admittanceController.kdp

# wrench = np.array([[0], [0], [0], [0], [0], [0]])
# testwrench = np.array([[0], [0], [0], [1], [2], [3]])
# t = 0
# t_end = 30
# dt = 1/500
timestamps = np.arange(0,timestamps1-stop-offset)

figf, axsf = plt.subplots(6,figsize=(4,3))
figf.suptitle('Actual TCP force')
axsf[0].plot(timestamps,p_force2[:,0],label='ref joint 1')
axsf[0].plot(timestamps,f_force2[:,0],label='adm joint 1')
axsf[0].legend(loc='upper right')
axsf[1].plot(timestamps,p_force2[:,1],label='ref joint 2')
axsf[1].plot(timestamps,f_force2[:, 1],label='adm joint 2')
axsf[1].legend(loc='upper right')
axsf[2].plot(timestamps,p_force2[:, 2],label='ref joint 3')
axsf[2].plot(timestamps,f_force2[:, 2],label='adm joint 3')
axsf[2].legend(loc='upper right')
axsf[3].plot(timestamps,p_force2[:, 3],label='ref joint 4')
axsf[3].plot(timestamps,f_force2[:, 3],label='adm joint 4')
axsf[3].legend(loc='upper right')
axsf[4].plot(timestamps,p_force2[:, 4],label='ref joint 5')
axsf[4].plot(timestamps,f_force2[:, 4],label='adm joint 5')
axsf[4].legend(loc='upper right')
axsf[5].plot(timestamps,p_force2[:, 5],label='ref joint 6')
axsf[5].plot(timestamps,f_force2[:, 5],label='adm joint 6')
axsf[5].legend(loc='upper right')
axsf[5].set(xlabel='time [s]', ylabel='force [N]')


# ref_pos = np.zeros([len(timestamps),3])
# ref_rot = np.zeros([len(timestamps),4])
# adm_pos = np.zeros([len(timestamps),3])
# adm_rot = np.zeros([len(timestamps),4])
# delta_pos = np.zeros([timestamps1,3])
# delta_rot = np.zeros([timestamps1,4])

#for i in range(timestamps):
#     t += dt

#     if t > 5 and t < 10:
#         wrench = np.array([[0], [0], [0], [10], [10], [10]])
#     elif t > 15 and t < 20: 
#         wrench = np.array([[10], [10], [10], [0], [0], [0]])
#     else:
#         wrench = np.array([[0], [0], [0], [0], [0], [0]])

#     diff_pos, diff_rot = admittanceController.step(wrench)
#     ref_pos[i,:] = copy.deepcopy(refTrajectory(t))
#     ref_rot[i,:] = copy.deepcopy(refOrientation(t))
#     adm_pos[i,:] = copy.deepcopy(refTrajectory(t)+diff_pos.T)
#     q_ref = quaternion.from_float_array(refOrientation(t))
#     q_diff = quaternion.from_float_array(diff_rot.T)
#     adm_rot[i,:] = copy.deepcopy(quaternion.as_float_array(q_ref*q_diff))
    # delta_pos[i,:] = cop(p_pos[i,:] - f_pos[i,:])
     #elta_rot[i,:] = copy.deepcopy(( p_rot[i,:] - f_rot[i,:]))
   
# plot position

#plt.plot(adm_pos[0,0:1000])

fig, axs = plt.subplots(3,figsize=(4,3))
fig.suptitle('Translation comparison, then force is applied')
axs[0].plot(timestamps,p_pos[:,0],label='ref x')
axs[0].plot(timestamps,f_pos[:,0],label='adm x')
axs[0].legend(loc='upper right')
axs[1].plot(timestamps,p_pos[:,1],label='ref y')
axs[1].plot(timestamps,f_pos[:, 1],label='adm y')
axs[1].legend(loc='upper right')
axs[2].plot(timestamps,p_pos[:, 2],label='ref z')
axs[2].plot(timestamps,f_pos[:, 2],label='adm z')
axs[2].legend(loc='upper right')
axs[2].set(xlabel='time [s]', ylabel='position [m]')


# deltafig, deltaaxs = plt.subplots(3,figsize=(4,3))
# deltafig.suptitle('Translation delta, then Force is applied')
# deltaaxs[0].plot(timestamps,delta_pos[:, 0],label='delta x')
# deltaaxs[0].legend(loc='upper right')
# deltaaxs[1].plot(timestamps,delta_pos[:, 1],label='delta y')
# deltaaxs[1].legend(loc='upper right')
# deltaaxs[2].plot(timestamps,delta_pos[:, 2],label='delta z')
# deltaaxs[2].legend(loc='upper right')
# deltaaxs[2].set(xlabel='time [s]', ylabel='position [m]')

#plot ref_pos and adm_pos in 3d
fig3 = plt.figure()
fig3.suptitle('Trajectory comparison shown in 3d')
ax3d = plt.axes(projection='3d')
ax3d.plot3D(p_pos[:,0],p_pos[:,1],p_pos[:,2],label='ref')
ax3d.plot3D(f_pos[:,0],f_pos[:,1],f_pos[:,2],label='adm')
ax3d.set_xlabel('X')
ax3d.set_ylabel('Y')
ax3d.set_zlabel('Z')
ax3d.set(xlabel='position [m]', ylabel='position [m]',zlabel='position [n]')


# plot orientation
# ofig, oaxs = plt.subplots(4,figsize=(4,3))
# ofig.suptitle('Orientation comparison, then wrench is applied')
# oaxs[0].plot(timestamps,p_rot[:,0],label='ref w')
# oaxs[0].plot(timestamps,f_rot[:,0],label='adm w')
# oaxs[0].legend(loc='upper right')
# oaxs[1].plot(timestamps,p_rot[:,1],label='ref x')
# oaxs[1].plot(timestamps,f_rot[:,1],label='adm x')
# oaxs[1].legend(loc='upper right')
# oaxs[2].plot(timestamps,p_rot[:,2],label='ref y')
# oaxs[2].plot(timestamps,f_rot[:,2],label='adm y')
# oaxs[2].legend(loc='upper right')
# oaxs[3].plot(timestamps,p_rot[:,3],label='ref z')
# oaxs[3].plot(timestamps,f_rot[:,3],label='adm z')
# oaxs[3].legend(loc='upper right')
# oaxs[3].set(xlabel='time [s]', ylabel='rotation')

# # plot delta orientation
# odelta, odeltaaxs = plt.subplots(4,figsize=(4,3))
# odelta.suptitle('Orientation delta, then wrench is applied')
# odeltaaxs[0].plot(timestamps,delta_rot[:,0],label='delta w')
# odeltaaxs[0].legend(loc='upper right')
# odeltaaxs[1].plot(timestamps,delta_rot[:,1],label='delta x')
# odeltaaxs[1].legend(loc='upper right')
# odeltaaxs[2].plot(timestamps,delta_rot[:,2],label='delta y')
# odeltaaxs[2].legend(loc='upper right')
# odeltaaxs[3].plot(timestamps,delta_rot[:,3],label='delta z')
# odeltaaxs[3].legend(loc='upper right')
# odeltaaxs[3].set(xlabel='time [s]', ylabel='rotation')

#plot angular velocity

# wfig, waxs = plt.subplots(3)
# wfig.suptitle('Vertically stacked subplots')
# waxs[0].plot(wdata[0,0:25000])
# waxs[1].plot(wdata[1,0:25000])
# waxs[2].plot(wdata[2,0:25000])





#wait for user to close plot
plt.grid()
plt.show()

# plt.plot(odata[1,0:25000])
# plt.plot(odata[2,0:25000])
# plt.plot(odata[3,0:25000])

# plt.show()

# norot = quat(1,0,0,0)
#finalrot = quat(odata[0,25000],odata[1,25000],odata[2,25000],odata[3,25000])

#finalrot.animate(start = norot)




#wait for key
#input("Press Enter to continue...")


