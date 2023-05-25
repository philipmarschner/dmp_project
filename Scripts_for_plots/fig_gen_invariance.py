from dmp import dmp_cartesian, dmp_joint, dmp_position
import numpy as np
import pandas as pd
import quaternion
import matplotlib.pyplot as plt
import sys



dt = 0.002
tau = 1
ts = np.arange(0, tau, dt)
demo_p = np.sin(np.linspace(0, -3/2*np.pi, len(ts)))
#demo_p = np.linspace(0, 1, len(ts))

#demo[500:600] = 0.5
n_bfs = 50

q = np.zeros((len(ts), 3))
q[:, 0] = demo_p
q[:, 1] = demo_p
q[:, 2] = demo_p



cs_alpha = -np.log(0.0001)


#fig_size = plt.rcParams["figure.figsize"]
#fig_size[0] = 4
#fig_size[1] = 3
#plt.rcParams["figure.figsize"] = fig_size
#dmp = dmp_position.PositionDMP(n_bfs, alpha=500, beta=100, cs_alpha=cs_alpha)
#dmp.train(q, ts, tau)
#temp = 0.4
#for i in range(1,9):
#    temp+= 0.2
#    temp = round(temp, 2)
#    ts_temp = np.arange(0, temp*tau, dt)
#    p, _, _ = dmp.rollout(ts_temp, temp*tau)
#    plt.plot(ts_temp,p[:,0], label = "$τ=$" + str(temp*tau))
#plt.legend()
#plt.xlabel('time [s]')
#plt.ylabel('y')
#plt.xlim([0, 3])
#plt.ylim([-1.1, 1.1])
#plt.grid()
#plt.tight_layout()
## set plot size
#plt.show()

fig_size = plt.rcParams["figure.figsize"]
fig_size[0] = 4
fig_size[1] = 3
plt.rcParams["figure.figsize"] = fig_size
dmp = dmp_position.PositionDMP(n_bfs, alpha=500, beta=100, cs_alpha=cs_alpha)
dmp.train(q, ts, tau)
k = -1.25
for i in range(1,10):
    k+= 0.25
    k = round(k, 2)
    #dmp.gp = k*(demo_p[-1] - demo_p[0])
    dmp.gp = k
    p, _, _ = dmp.rollout(ts, tau)
    plt.plot(ts,p[:,1], label = "$g=$" + str(k))
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('y')
plt.xlim([0, 1.70])
#plt.ylim([-10, 10])
plt.grid()
plt.tight_layout()
# set plot size



plt.show()
