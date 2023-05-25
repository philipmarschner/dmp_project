from admittance import Admitance
import numpy as np
import matplotlib.pyplot as plt
import quaternion
import copy

def refTrajectory(t):
    return np.array([0.3*np.sin(t/3), 0.3*np.sin(t/3)*np.cos(t/3), 0.1*np.sin(t)])

def refOrientation(t):
    return np.array([1,0,0,0])





Tc = np.array([[1,0,0,0],[0,1,0,0.1],[0,0,1,0.2],[0,0,0,1]])
admittanceController = Admitance(robot = None, kp = 100, ko = 200,Tc = Tc, kdp = 100, kdo=100)

tempread = admittanceController.kdp

wrench = np.array([[0], [0], [0], [0], [0], [0]])
testwrench = np.array([[0], [0], [0], [1], [2], [3]])
t = 0
t_end = 23
dt = 1/500
timestamps = np.arange(0,t_end,dt)

ref_pos = np.zeros([len(timestamps),3])
ref_rot = np.zeros([len(timestamps),4])
adm_pos = np.zeros([len(timestamps),3])
adm_rot = np.zeros([len(timestamps),4])
delta_pos = np.zeros([len(timestamps),3])
delta_rot = np.zeros([len(timestamps),4])


for i in range(len(timestamps)):
    t += dt
    if t > 5 and t < 10:
        wrench = np.array([[10], [10], [10], [0], [0], [0]])
    elif t > 12 and t < 17: 
        wrench = np.array([[0], [0], [0], [10], [10], [10]])
    else:
        wrench = np.array([[0], [0], [0], [0], [0], [0]])

    diff_pos, diff_rot = admittanceController.step(wrench)
    ref_pos[i,:] = copy.deepcopy(refTrajectory(t))
    ref_rot[i,:] = copy.deepcopy(refOrientation(t))
    adm_pos[i,:] = copy.deepcopy(refTrajectory(t)+diff_pos.T)
    q_ref = quaternion.from_float_array(refOrientation(t))
    q_diff = quaternion.from_float_array(diff_rot.T)
    adm_rot[i,:] = copy.deepcopy(quaternion.as_float_array(q_ref*q_diff))
    delta_pos[i,:] = copy.deepcopy(diff_pos.T)
    delta_rot[i,:] = copy.deepcopy(diff_rot.T)
    
# plot position


#plot ref_pos and adm_pos in 3d
fig3 = plt.figure(figsize=(4,3))
ax3d = plt.axes(projection='3d')
ax3d.plot3D(ref_pos[:,0],ref_pos[:,1],ref_pos[:,2],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
ax3d.plot3D(adm_pos[:,0],adm_pos[:,1],adm_pos[:,2],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
ax3d.legend(loc='upper right',facecolor='white')
ax3d.set(xlabel='x [m]', ylabel='y [m]',zlabel='z [m]')
ax3d.set_facecolor('white')

fig, axs = plt.subplots(3,1,figsize=(4,3))
axs[0].plot(timestamps,ref_pos[:,0],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
axs[0].plot(timestamps,adm_pos[:,0],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
axs[0].set(ylabel='x [m]')
axs[1].plot(timestamps,ref_pos[:,1],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
axs[1].plot(timestamps,adm_pos[:,1],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
axs[1].set(ylabel='y [m]')
axs[2].plot(timestamps,ref_pos[:, 2],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
axs[2].plot(timestamps,adm_pos[:, 2],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
axs[2].set(xlabel='time [s]', ylabel='z [m]')
for ax in axs:
    ax.legend(loc='upper right',facecolor='white')
    ax.set_facecolor('white')
    ax.grid(True)
    for spine in ax.spines.values():
        spine.set_edgecolor('grey')

plt.tight_layout()


deltafig, deltaaxs = plt.subplots(3,1,figsize=(4,3))
#plt.suptitle('Translation delta, then Force is applied')
deltaaxs[0].plot(timestamps,delta_pos[:, 0],label='$p_d-p_c$',linestyle='-',linewidth=2,color='#1f77b4')
deltaaxs[0].set(ylabel='x [m]')
deltaaxs[1].plot(timestamps,delta_pos[:, 1],label='$p_d-p_c$',linestyle='-',linewidth=2,color='#1f77b4')
deltaaxs[1].set(ylabel='y [m]')
deltaaxs[2].plot(timestamps,delta_pos[:, 2],label='$p_d-p_c$',linestyle='-',linewidth=2,color='#1f77b4')
deltaaxs[2].set(xlabel='time [s]', ylabel='z [m]')
for ax in deltaaxs:
    ax.legend(loc='upper right',facecolor='white')
    ax.set_facecolor('white')
    ax.grid(True)
    for spine in ax.spines.values():
        spine.set_edgecolor('grey')

plt.tight_layout()

# plot orientation
ofig, oaxs = plt.subplots(4,1,figsize=(4,4))
#plt.suptitle('Orientation comparison, then wrench is applied')
oaxs[0].plot(timestamps,ref_rot[:,0],label=r'$q_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
oaxs[0].plot(timestamps,adm_rot[:,0],label=r'$q_c$',zorder=1,linestyle='-', linewidth=2,color='#ff7f0e')
oaxs[0].set(ylabel='$q_0$')
oaxs[1].plot(timestamps,ref_rot[:,1],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
oaxs[1].plot(timestamps,adm_rot[:,1],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
oaxs[1].set(ylabel='$q_1$')
oaxs[2].plot(timestamps,ref_rot[:,2],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
oaxs[2].plot(timestamps,adm_rot[:,2],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
oaxs[2].set(ylabel='$q_2$')
oaxs[3].plot(timestamps,ref_rot[:,3],label=r'$p_d$',zorder=2,linestyle='--',linewidth=1,color='#1f77b4')
oaxs[3].plot(timestamps,adm_rot[:,3],label=r'$p_c$',zorder=1,linestyle='-',linewidth=2,color='#ff7f0e')
oaxs[3].set(xlabel='time [s]', ylabel='$q_3$')
for ax in oaxs:
    ax.legend(loc='upper left',facecolor='white')
    ax.set_facecolor('white')
    ax.grid(True)
    for spine in ax.spines.values():
        spine.set_edgecolor('grey')
plt.tight_layout()

# plot delta orientation
odelta, odeltaaxs = plt.subplots(4,1,figsize=(4,4))
#plt.suptitle('Orientation delta, then wrench is applied')
odeltaaxs[0].plot(timestamps,delta_rot[:,0],label=r'$\epsilon_{cd}$',zorder=1,linestyle='-', linewidth=2,color='#1f77b4')
odeltaaxs[0].set(ylabel='$q_0$')
odeltaaxs[1].plot(timestamps,delta_rot[:,1],label=r'$\epsilon_{cd}$',zorder=1,linestyle='-', linewidth=2,color='#1f77b4')
odeltaaxs[1].set(ylabel='$q_1$')
odeltaaxs[2].plot(timestamps,delta_rot[:,2],label=r'$\epsilon_{cd}$',zorder=1,linestyle='-', linewidth=2,color='#1f77b4')
odeltaaxs[2].set(ylabel='$q_2$')
odeltaaxs[3].plot(timestamps,delta_rot[:,3],label=r'$\epsilon_{cd}$',zorder=1,linestyle='-', linewidth=2,color='#1f77b4')
odeltaaxs[3].set(xlabel='time [s]', ylabel='$q_3$')
for ax in odeltaaxs:
    ax.legend(loc='upper left',facecolor='white')
    ax.set_facecolor('white')
    ax.grid(True)
    for spine in ax.spines.values():
        spine.set_edgecolor('grey')
plt.tight_layout()
plt.show()
exit()
