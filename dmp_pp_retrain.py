from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plot = False

# Parameters
demo_filename = "recorded_data.csv"
demo = pd.read_csv(demo_filename, delimiter=",")


p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()


t_steps = len(p)
dt = 1.0/500.0  # 2ms


tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 100


#MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
MP.imitate_path(x_des=p)

collision = False

#p_out, _, _, _ = MP.rollout()
p_out = np.zeros(p.shape)
for i in range(len(ts)):
    p_temp, _, _ = MP.step()
    p_out[i,:] = p_temp

    if ts[i] > 0.5 and not collision:
        collision = True
        s_collision_start = MP.cs.s
        t_collision_start = ts[i]
        #s1_bar = s_collision_start + MP.width

    if ts[i] > 1 and collision:
        collision = False
        s_collision_end = MP.cs.s
        t_collision_end = ts[i]
        #s0_bar = s_collision_end - MP.width

       
xnew = None
MP.retrain(x_des = xnew, t0 = t_collision_start, t1 = t_collision_end, s0 = s_collision_start, s1 = s_collision_end)

if plot:
    # Plot the results
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], label=r'Demonstration', linewidth=2)
    ax1.plot3D(p_out[:, 0], p_out[:, 1], p_out[:, 2], label=r'DMP', linewidth=2)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    plt.title('Cartesian-space DMP (Position)')
    plt.legend()

    fig2, axs = plt.subplots(3, 1)
    p = p[0:len(p_out),:]
    ts = ts[0:len(p_out)]
    axs[0].plot(ts, p[:, 0], label='Demo x')
    axs[0].plot(ts, p_out[:, 0], label='DMP x')
    axs[0].legend()
    axs[1].plot(ts, p[:, 1], label='Demo y')
    axs[1].plot(ts, p_out[:, 1], label='DMP y')
    axs[1].legend()
    axs[2].plot(ts, p[:, 2], label='Demo z')
    axs[2].plot(ts, p_out[:, 2], label='DMP z')
    axs[2].legend()
    plt.suptitle('Cartesian-space DMP (Position)')

    plt.show()