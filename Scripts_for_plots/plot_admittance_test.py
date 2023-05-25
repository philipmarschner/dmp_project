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

demo_file ="/home/phmar/Documents/dmp_project/momentum_observer/May_22_log4_plane_demonstration.csv"
demo = pd.read_csv(demo_file, delimiter=",")
force_file = "/home/phmar/Documents/dmp_project/momentum_observer/May_22_log_compliance_demonstration.csv"
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


timestamps = np.arange(0,timestamps1-stop-offset)
time_converted = timestamps*0.002

figf, axsf = plt.subplots(3,2,figsize=(8,4))
axsf[0,0].plot(time_converted,p_force2[:,0],label='No ext. f')
axsf[0,0].plot(time_converted,f_force2[:,0],label='With ext. f')
axsf[0,0].legend(loc='lower left')
axsf[0,0].set(ylabel='$f_x$ [N]')
axsf[0,0].grid(True)
axsf[1,0].plot(time_converted,p_force2[:,1],label='No ext. f')
axsf[1,0].plot(time_converted,f_force2[:,1],label='with ext. f')
axsf[1,0].legend(loc='lower left')
axsf[1,0].set( ylabel='$f_y$ [N]')
axsf[1,0].grid(True)
axsf[2,0].plot(time_converted,p_force2[:, 2],label='No ext. f')
axsf[2,0].plot(time_converted,f_force2[:, 2],label='With ext. f')
axsf[2,0].legend(loc='lower left')
axsf[2,0].set(xlabel='time [s]', ylabel='$f_z$ [N]')
axsf[2,0].grid(True)
axsf[0,1].plot(time_converted,p_force2[:, 3],label='No ext. m')
axsf[0,1].plot(time_converted,f_force2[:, 3],label='With ext. m')
axsf[0,1].legend(loc='lower left')
axsf[0,1].set(ylabel='$m_x$ [Nm]')
axsf[0,1].grid(True)
axsf[1,1].plot(time_converted,p_force2[:, 4],label='No ext. m')
axsf[1,1].plot(time_converted,f_force2[:, 4],label='With ext. m')
axsf[1,1].legend(loc='upper left')
axsf[1,1].set(ylabel='$m_y$ [Nm]')
axsf[1,1].grid(True)
axsf[2,1].plot(time_converted,p_force2[:, 5],label='No ext. m')
axsf[2,1].plot(time_converted,f_force2[:, 5],label='With ext. m')
axsf[2,1].legend(loc='lower left')
axsf[2,1].set(xlabel='time [s]', ylabel='$m_z$ [Nm]')
axsf[2,1].grid(True)
#add title
figf.suptitle('Actual TCP force comparison with and without external wrench')
plt.tight_layout()



# fig, axs = plt.subplots(3,figsize=(4,3))
# fig.suptitle('Translation comparison, then force is applied')
# axs[0].plot(time_converted,p_pos[:,0],label='ref x')
# axs[0].plot(time_converted,f_pos[:,0],label='adm x')
# axs[0].legend(loc='upper right')
# axs[1].plot(time_converted,p_pos[:,1],label='ref y')
# axs[1].plot(time_converted,f_pos[:, 1],label='adm y')
# axs[1].legend(loc='upper right')
# axs[2].plot(time_converted,p_pos[:, 2],label='ref z')
# axs[2].plot(time_converted,f_pos[:, 2],label='adm z')
# axs[2].legend(loc='upper right')
# axs[2].set(xlabel='time [s]', ylabel='position [m]')

fig, axs = plt.subplots(3,1,figsize=(4,3))
axs[0].plot(time_converted,p_pos[:,0],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1)
axs[0].plot(time_converted,f_pos[:,0],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2)
axs[0].set(ylabel='x [m]')
axs[1].plot(time_converted,p_pos[:,1],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1)
axs[1].plot(time_converted,f_pos[:,1],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2)
axs[1].set(ylabel='y [m]')
axs[1].legend(loc='upper left',facecolor='white')
axs[2].plot(time_converted,p_pos[:, 2],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1)
axs[2].plot(time_converted,f_pos[:, 2],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2)
axs[2].set(xlabel='time [s]', ylabel='z [m]')
for ax in axs:
    ax.set_facecolor('white')
    ax.grid(True)
    for spine in ax.spines.values():
        spine.set_edgecolor('grey')
plt.tight_layout()


#plot ref_pos and adm_pos in 3d
# fig3 = plt.figure()
# fig3.suptitle('Trajectory comparison shown in 3d')
# ax3d = plt.axes(projection='3d')
# ax3d.plot3D(p_pos[:,0],p_pos[:,1],p_pos[:,2],label='ref')
# ax3d.plot3D(f_pos[:,0],f_pos[:,1],f_pos[:,2],label='adm')
# ax3d.set_xlabel('X')
# ax3d.set_ylabel('Y')
# ax3d.set_zlabel('Z')
# ax3d.set(xlabel='position [m]', ylabel='position [m]',zlabel='position [n]')

fig3 = plt.figure(figsize=(4,3))
ax3d = plt.axes(projection='3d')
ax3d.plot3D(p_pos[:,0],p_pos[:,1],p_pos[:,2],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1)
ax3d.plot3D(f_pos[:,0],f_pos[:,1],f_pos[:,2],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2)
ax3d.legend(loc='upper right',facecolor='white')
ax3d.set(xlabel='x [m]', ylabel='y [m]',zlabel='z [m]')
ax3d.set_facecolor('white')


plt.show()

