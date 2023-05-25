from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy

plot = True

# Parameters
demo_filename = "momentum_observer/MAY_18_recording.csv"
demo = pd.read_csv(demo_filename, delimiter=",")
full_demo_fn = "May_18_log3_demonstration.csv"
reteach_filename = "May_18_log3_retrained_poses.csv"
full_demo = pd.read_csv(full_demo_fn, delimiter=",")
reteach = pd.read_csv(reteach_filename, delimiter=",")
#demo_filename = "demonstration.csv"
#demo = pd.read_csv(demo_filename, delimiter=" ")


p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
force = full_demo[['actual_TCP_force_0', 'actual_TCP_force_1', 'actual_TCP_force_2']].to_numpy()
#p_reteach = reteach[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
p_reteach  =  reteach.to_numpy()
print("p",p.shape)
print("p reteach",p_reteach.shape)

t_steps = len(p)
dt = 1.0/500.0  # 2ms


tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 300


MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
original_target = MP.imitate_path(x_des=p)




collision = False
p_retrained = []
p_retrained_plot = []
p_original_interval = []
f_logged = []


p_out = np.zeros(p.shape)
col_start = 3510*dt
col_end = col_start + 1750*dt
j = 0
for i in range(len(ts)):
    p_temp, _, _ = MP.step()
    p_out[i,:] = p_temp

    if ts[i] > col_start and ts[i]<=col_end and not collision:
        collision = True
        s_collision_start = MP.cs.s
        s0_index = i
        t_collision_start = ts[i]

    if ts[i] > col_end and collision:
        collision = False

    if collision:
        s_collision_end = MP.cs.s
        t_collision_end = ts[i]
        s1_index = i
        p_retrained.append(p_reteach[j,:].tolist())
        
        j += 1
        #p_retrained.append((p[i,:] + np.array([0,0,0.1])).tolist())
        #p_original_interval.append((original_target[:,i]).tolist())
        

    else:
        #p_retrained_plot.append((p[i,:]).tolist())
        pass


xnew = np.array(p_retrained)
#x_original_interval = np.array(p_original_interval)
p_retrained_plot_array = np.array(p_retrained_plot)

temp_w = copy.deepcopy(MP.w)

new_target = MP.retrain(x_new = xnew, f_target_original = original_target, t0 = t_collision_start, t1 = t_collision_end, s0 = s_collision_start, s1 = s_collision_end)

print("new_target.shape: ", new_target.shape)
#print("x_original_interval.shape: ", x_original_interval.shape)



#plt.plot(ts[0:new_target.shape[1]],new_target[1,:], label='new_target')
#plt.plot(ts[0:new_target.shape[1]],x_original_interval[:,1], label='original_target')
#plt.plot(ts[0:new_target.shape[1]],new_target[1,:]-x_original_interval[:,1], label='p_retrained_plot_array')
#plt.show()




retrained_MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier',w = MP.w, x_0 = MP.x_0, x_goal = MP.x_goal)



p_out_retrained = np.zeros(p.shape)
for i in range(len(ts)):
    p_temp_retrained, _, _ = retrained_MP.step()
    p_out_retrained[i,:] = p_temp_retrained

f_logged = np.array(f_logged)

if plot:
    #Plot the results
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], label=r'Demonstration', linewidth=1, linestyle='--',zorder=3)
    ax1.plot3D(p_out[:, 0], p_out[:, 1], p_out[:, 2], label=r'DMP original', linewidth=2, zorder=2)
    ax1.plot3D(p_out_retrained[:, 0], p_out_retrained[:, 1], p_out_retrained[:, 2], label=r'DMP adapted', linewidth=2, zorder=1)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    plt.title('Cartesian-space DMP (Position)')
    plt.legend(loc='upper right')

    fig2, axs = plt.subplots(3, 2, figsize=(9,5))
    axs[0,0].plot(ts, p[:, 0], label='Demo x',linestyle="--",zorder=3)
    axs[0,0].plot(ts, p_out[:, 0], label='$DMP_o x$',zorder=1,linewidth=3)
    axs[0,0].plot(ts, p_out_retrained[:, 0], label='$DMP_a x$',zorder=2)
    #axs[0,0].legend(loc='lower left')
    axs[0,0].grid()
    axs[0,0].set_ylabel('X [m]')
    axs[1,0].plot(ts, p[:, 1], label='Demo',linestyle="--",zorder=3)
    axs[1,0].plot(ts, p_out[:, 1], label='$DMP_o$',zorder=1,linewidth=3)
    axs[1,0].plot(ts, p_out_retrained[:, 1], label='$DMP_a$',zorder=2)
    axs[1,0].legend(loc='upper left')
    axs[1,0].grid()
    axs[1,0].set_ylabel('Y [m]')
    axs[2,0].plot(ts, p[:, 2], label='Demo z',linestyle="--",zorder=3)
    axs[2,0].plot(ts, p_out[:, 2], label='$DMP_o$ z',zorder=1,linewidth=3)
    axs[2,0].plot(ts, p_out_retrained[:, 2], label='$DMP_a$ z', zorder=2)
    #axs[2,0].legend(loc='upper left')
    axs[2,0].grid()
    axs[2,0].set_ylabel('Z [m]')
    axs[2,0].set_xlabel('Time [s]')
    #axs[0,1].plot(ts, p_out[:, 0]  - p[:, 0], label='$\Delta(DMP_o, Demo)$',color='orange')
    axs[0,1].plot(ts, p_out_retrained[:, 0] - p_out[:, 0], label='$\Delta(DMP_a, DMP_o)$')
    #axs[0,1].legend(loc='lower right')
    axs[0,1].grid()
    axs[0,1].set_ylabel('X [m]')
    #axs[1,1].plot(ts, p_out[:, 1] - p[:, 1], label='$\Delta(DMP_o, Demo)$',color='orange')
    axs[1,1].plot(ts, p_out_retrained[:, 1]- p_out[:, 1], label='$\Delta(DMP_a, DMP_o)$')
    axs[1,1].legend(loc='lower left')
    axs[1,1].grid()
    axs[1,1].set_ylabel('Y [m]')
    #axs[2,1].plot(ts, p_out[:, 2]- p[:, 2], label='$\Delta(DMP_o, Demo)$',color='orange')
    axs[2,1].plot(ts, p_out_retrained[:, 2]- p_out[:, 2], label='$\Delta(DMP_a, DMP_o)$')
    #axs[2,1].legend(loc='upper left')
    axs[2,1].grid()
    axs[2,1].set_ylabel('Z [m]')
    axs[2,1].set_xlabel('Time [s]')
    plt.suptitle('Cartesian-space DMP (Position)')
    plt.tight_layout()

    temp_ts = np.linspace(col_start,col_end,2196-50)
    fig3, axs = plt.subplots(3, figsize=(5,4))
    axs[0].plot(temp_ts,force[18779:18779+2196-50, 0],zorder=3)
    axs[0].set_ylabel('$F_x$ [N]')
    axs[0].grid()
    axs[1].plot(temp_ts,force[18779:18779+2196-50, 1],zorder=3)
    axs[1].set_ylabel('$F_y$ [N]')
    axs[1].grid()
    axs[2].plot(temp_ts,force[18779:18779+2196-50, 2],zorder=3)
    axs[2].set_ylabel('$F_z$ [N]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid()
    plt.tight_layout()
    plt.show()
