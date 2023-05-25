from dmp import dmp_cartesian, dmp_joint
import numpy as np
import pandas as pd
import quaternion
import matplotlib.pyplot as plt
import sys
#from stream_path import stream_traj




plot = True




# Read Demonstration and generate time vector
#demo_filename = "demonstration.csv"
demo_filename = "finaltest.csv"
demo = pd.read_csv(demo_filename, delimiter=",")

q = demo[['actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']].to_numpy()
p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
aa = demo[['actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()

dt = 0.002
tau = len(demo) * dt
ts = np.arange(0, tau, dt)
n_bfs = 100


# Ensure that the orientations are formatted properly
for i in range(1, len(aa)):
    if np.dot(aa[i], aa[i - 1]) < 0:
        aa[i] *= -1

quats = quaternion.from_rotation_vector(aa)

# Ensure that the quaternions do not flip sign
for i in range(1, len(quats)):
    q0 = quats[i - 1]
    q1 = quats[i]

    if np.dot(q0.vec, q1.vec) < 0:
        quats[i] *= -1

# Train DMP
cs_alpha = -np.log(0.0001)
dmp = dmp_cartesian.CartesianDMP(10, alpha=500, beta=100, cs_alpha=cs_alpha)
dmp.train(p, quats, ts, tau)

dmp2 = dmp_cartesian.CartesianDMP(50, alpha=500, beta=100, cs_alpha=cs_alpha)
dmp2.train(p, quats, ts, tau)

p_out, dp_out, _, _, _, _ = dmp.rollout(ts, tau)
p_out_retrained, _, _, _, _, _ = dmp2.rollout(ts, tau)

if plot:
    #Plot the results
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], label=r'Demonstration', linewidth=2)
    ax1.plot3D(p_out[:, 0], p_out[:, 1], p_out[:, 2], label=r'DMP with 20 bfs', linewidth=2)
    ax1.plot3D(p_out_retrained[:, 0], p_out_retrained[:, 1], p_out_retrained[:, 2], label=r'DMP with 50 bfs', linewidth=2)
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
    axs[0].plot(ts, p_out[:, 0], label='DMP x with 20 bfs')
    axs[0].plot(ts, p_out_retrained[:, 0], label='DMP x with 50 bfs')
    #axs[0].plot(ts, p_retrained_plot_array[:, 0], label='DMP retrained x')
    axs[0].legend(loc='upper right')
    axs[1].plot(ts, p[:, 1], label='Demo y')
    axs[1].plot(ts, p_out[:, 1], label='DMP y with 20 bfs')
    axs[1].plot(ts, p_out_retrained[:, 1], label='DMP y with 20 bfs')
    axs[1].legend(loc='lower right')
    axs[2].plot(ts, p[:, 2], label='Demo z')
    axs[2].plot(ts, p_out[:, 2], label='DMP z with 20 bfs')
    axs[2].plot(ts, p_out_retrained[:, 2], label='DMP z with 50 bfs')
    axs[2].legend(loc='upper right')
    plt.suptitle('Cartesian-space DMP (Position)')

    fig3, ax3 = plt.subplots(3, 1)
    ts = ts[0:len(p_out_retrained)]
    ax3[0].plot(ts, p[:, 0]- p_out[:, 0], label='Difference DMP x with 20 bfs')
    ax3[0].plot(ts, p[:, 0]- p_out_retrained[:, 0], label='Difference DMP x with 50 bfs')
    ax3[0].legend(loc='lower right')
    ax3[1].plot(ts, p[:, 1]- p_out[:,1], label='Difference DMP y with 20 bfs')
    ax3[1].plot(ts, p[:, 1]- p_out_retrained[:,1], label='Difference DMP y with 50 bfs')
    ax3[1].legend(loc='lower right')
    ax3[2].plot(ts, p[:, 2]- p_out[:, 2], label='Difference DMP z with 20 bfs')
    ax3[2].plot(ts, p[:, 2]- p_out_retrained[:, 2], label='Difference DMP z with 50 bfs')
    ax3[2].legend(loc='upper right')
    plt.suptitle('Difference between demonstration and DMP (Position)')

    plt.show()
