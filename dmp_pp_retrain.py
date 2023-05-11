from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import copy

plot = True

# Parameters
demo_filename = "live_demo_log_admittance_control.csv"
demo = pd.read_csv(demo_filename, delimiter=",")
reteach_filename = "live_demo_reteach_log_admittance_control.csv"
reteach = pd.read_csv(reteach_filename, delimiter=",")
#demo_filename = "demonstration.csv"
#demo = pd.read_csv(demo_filename, delimiter=" ")


p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
p_reteach = reteach[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
print("p",p.shape)
print("p reteach",p_reteach.shape)

t_steps = len(p)
dt = 1.0/500.0  # 2ms


tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 100


MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
original_target = MP.imitate_path(x_des=p)


collision = False
p_retrained = []
p_retrained_plot = []
p_original_interval = []


p_out = np.zeros(p.shape)
col_start = 2
col_end = 6
j = 1400
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



if plot:
    #Plot the results
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], label=r'Demonstration', linewidth=2)
    ax1.plot3D(p_out[:, 0], p_out[:, 1], p_out[:, 2], label=r'DMP', linewidth=2)
    ax1.plot3D(p_out_retrained[:, 0], p_out_retrained[:, 1], p_out_retrained[:, 2], label=r'DMP retrained', linewidth=2)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    plt.title('Cartesian-space DMP (Position)')
    plt.legend(loc='upper right')

    fig2, axs = plt.subplots(3, 1)
    p = p[0:len(p_out_retrained),:]
    p_out = p_out[0:len(p_out_retrained),:]
    ts = ts[0:len(p_out_retrained)]
    axs[0].plot(ts, p[:, 0], label='Demo x')
    axs[0].plot(ts, p_out[:, 0], label='DMP x')
    axs[0].plot(ts, p_out_retrained[:, 0], label='DMP retrained x')
    #axs[0].plot(ts, p_retrained_plot_array[:, 0], label='DMP retrained x')
    axs[0].legend(loc='upper right')
    axs[1].plot(ts, p[:, 1], label='Demo y')
    axs[1].plot(ts, p_out[:, 1], label='DMP y')
    axs[1].plot(ts, p_out_retrained[:, 1], label='DMP retrained y')
    axs[1].legend(loc='lower right')
    axs[2].plot(ts, p[:, 2], label='Demo z')
    axs[2].plot(ts, p_out[:, 2], label='DMP z')
    axs[2].plot(ts, p_out_retrained[:, 2], label='DMP retrained z')
    axs[2].legend(loc='upper right')
    plt.suptitle('Cartesian-space DMP (Position)')

    fig3, ax3 = plt.subplots(3, 1)
    ts = ts[0:len(p_out_retrained)]
    ax3[0].plot(ts, p_out[:, 0]- p_out_retrained[:, 0], label='Difference DMP x')
    ax3[0].legend(loc='upper right')
    ax3[1].plot(ts, p_out[:, 1]- p_out_retrained[:,1], label='Difference DMP y')
    ax3[1].legend(loc='lower right')
    ax3[2].plot(ts, p_out[:, 2]- p_out_retrained[:, 2], label='Difference DMP z')
    ax3[2].legend(loc='lower right')
    plt.suptitle('Difference when retrained')

    plt.show()
