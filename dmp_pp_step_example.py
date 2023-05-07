from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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


p_out = np.zeros(p.shape)
for i in range(len(ts)):
    p_temp, _, _ = MP.step()
    p_out[i,:] = p_temp

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
print('p_out', p_out.shape)
print('p', p.shape)
print('ts', ts.shape)
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