from dmp import dmp_cartesian, dmp_joint, dmp_position
import numpy as np
import pandas as pd
import quaternion
import matplotlib.pyplot as plt
import sys



dt = 0.002
tau = 2
ts = np.arange(0, tau, dt)
demo_a = np.sin(np.linspace(0, 2*np.pi, len(ts)))
# integrate to get demo_v
demo_v = np.zeros(len(ts))
for i in range(1, len(ts)):
    demo_v[i] = demo_v[i-1] + demo_a[i] * dt

# integrate to get demo_p
demo_p = np.zeros(len(ts))
for i in range(1, len(ts)):
    demo_p[i] = demo_p[i-1] + demo_v[i] * dt

#demo[500:600] = 0.5
n_bfs = 5

q = np.zeros((len(ts), 3))
q[:, 0] = demo_p
q[:, 1] = demo_p
q[:, 2] = demo_p



cs_alpha = -np.log(0.0001)
dmp_q = dmp_position.PositionDMP(n_bfs, alpha=500, beta=100, cs_alpha=cs_alpha)
dmp_q.train(q, ts, tau)
q_out, dq_out, ddq_out = dmp_q.rollout(ts, tau)

x = dmp_q.cs.rollout(ts, tau)  # Integrate canonical system
psi = np.zeros((len(ts), n_bfs))
for i in range(len(x)):
    for j in range(n_bfs):
        psi[i,j] = np.exp(-dmp_q.h[j] * (x[i] - dmp_q.c[j]) ** 2)

c = np.reshape(dmp_q.c, [n_bfs , 1])
w = np.reshape(dmp_q.h, [n_bfs ,1 ])
xi = w * (x - c) * (x - c)
psi_set = np.exp(- xi)

psi_w = np.zeros((len(ts), n_bfs))
for i in range(len(x)):
    for j in range(n_bfs):
        psi_w[i,j] = dmp_q.w[0,j]* psi[i, j]

fig_size = plt.rcParams["figure.figsize"]
fig_size[0] = 3
fig_size[1] = 2
plt.rcParams["figure.figsize"] = fig_size
plt.plot(ts,x)
plt.xlabel('Time [s]')
plt.ylabel('x')
plt.xlim([0, 2])
plt.ylim([0, 1])
plt.grid()
# set plot size

plt.tight_layout()


fig1, axs = plt.subplots(3, 2)
fig1.figsize = (5,5)

axs[0,0].plot(ts, q[:, 0])
axs[0,0].plot(ts, q_out[:, 0])
axs[0,0].set_ylabel('$y$')

axs[1,0].plot(ts, demo_a)
axs[1,0].plot(ts, ddq_out[:, 0])
axs[1,0].set_ylabel('$\ddot{y}$')

for i in range(n_bfs):
    axs[2,0].plot(ts, psi[:, i])
axs[2,0].set_ylabel('$\mathbf{\psi}_i(x)$')
axs[2,0].set_xlabel('Time [s]')

axs[0,1].plot(ts, demo_v)
axs[0,1].plot(ts, dq_out[:, 0])
axs[0,1].set_ylabel('$\dot{y}$')

axs[1, 1].plot(ts, x, label='Demo q4')
axs[1,1].set_ylabel('$x$')

for i in range(n_bfs):
    axs[2,1].plot(ts, psi_w[:, i])
axs[2,1].set_ylabel('$w_i\mathbf{\psi}_i(x)$')
axs[2,1].set_xlabel('Time [s]')

plt.tight_layout()


plt.show()
