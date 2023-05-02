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


plt.show()