from dmp_pp.dmp.dmp_cartesian import DMPs_cartesian as dmp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Parameters
demo_filename = "recorded_data.csv"
demo = pd.read_csv(demo_filename, delimiter=",")

q = demo[['actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']].to_numpy()
p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
aa = demo[['actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()

dt = 1.0/500.0  # 2ms
tau = len(demo) * dt
ts = np.arange(0, tau, dt)
n_bfs = 100
cs_alpha = -np.log(0.0001)

MP = dmp(n_dmps = 3, n_bfs=n_bfs, dt=dt, T = tau, alpha_s = cs_alpha, basis='mollifier')
MP.imitate_path(p)
p_out, _, _, _ = MP.rollout()

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