from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Parameters
demo_filename = "live_demo_reteach_log_admittance_control.csv"
demo = pd.read_csv(demo_filename, delimiter=",")


p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()



t_steps = len(p)
dt = 1.0/500.0  # 2ms


tau = len(p) * dt
ts = np.arange(0, tau, dt)
n_dmp = p.shape[1]
n_bfs = 20


#MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
MP.imitate_path(x_des=p)
MP2 = dmp(n_dmps = n_dmp, n_bfs=100, dt = dt, T = ts[-1], basis='mollifier')
MP2.imitate_path(x_des=p)


p_out = np.zeros(p.shape)
p_out2 = np.zeros(p.shape)
for i in range(len(ts)):
    #p_temp, _, _ = MP.step()
    #p_out[i,:] = p_temp
    p_temp2, _, _ = MP2.step()
    p_out2[i,:] = p_temp2

# Plot the results
fig1 = plt.figure()
ax1 = plt.axes(projection='3d')
ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], label=r'Demonstration', linewidth=2)
#ax1.plot3D(p_out[:, 0], p_out[:, 1], p_out[:, 2], label=r'DMP 20 bfs', linewidth=2)
ax1.plot3D(p_out2[:, 0], p_out2[:, 1], p_out2[:, 2], label=r'DMP 100 bfs ', linewidth=2)
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
#ax1.axes.set_xlim3d(left=-0.8, right=0.0)
#ax1.axes.set_ylim3d(bottom=-1.0, top=0.0)
#ax1.axes.set_zlim3d(bottom=0.0, top=0.1)
plt.title('Cartesian-space DMP (Position)')
plt.legend()
print('p_out', p_out.shape)
print('p', p.shape)
print('ts', ts.shape)
fig2, axs = plt.subplots(3, 1)
p = p[0:len(p_out),:]
ts = ts[0:len(p_out)]
axs[0].plot(ts, p[:, 0], label='Demo')
#axs[0].plot(ts, p_out[:, 0], label='DMP 20 bfs')
axs[0].plot(ts, p_out2[:, 0], label='DMP 100 bfs')
axs[0].set_ylabel('X [m]')
axs[0].legend(loc='upper right')
axs[1].plot(ts, p[:, 1], label='Demo')
#axs[1].plot(ts, p_out[:, 1], label='DMP 20 bfs')
axs[1].plot(ts, p_out2[:, 1], label='DMP 100 bfs')
axs[1].set_ylabel('Y [m]')
axs[1].legend(loc='lower right')
axs[2].plot(ts, p[:, 2], label='Demo')
#axs[2].plot(ts, p_out[:, 2], label='DMP 20 bfs')
axs[2].plot(ts, p_out2[:, 2], label='DMP 100 bfs')
axs[2].set_ylabel('Z [m]')
axs[2].legend(loc='upper right')
axs[2].set_xlabel('Time [s]')
plt.suptitle('Cartesian-space DMP (Position)')

plt.show()