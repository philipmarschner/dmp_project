import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg
import pandas

import rtde_control
import rtde_receive


import matplotlib.pyplot as plt
import time
from scipy.fft import fft, ifft, fftfreq

from scipy import signal

from scipy.fft import fftshift
from numpy import genfromtxt
from livefilter import LiveLFilter
#my_data = genfromtxt('../FT_data_log6.csv', delimiter=',')
demo = pandas.read_csv('./log_admittance_control_dmp_nochair.csv', delimiter=',')

q = demo[['actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']].to_numpy()
qd = demo[['actual_qd_0', 'actual_qd_1', 'actual_qd_2', 'actual_qd_3', 'actual_qd_4', 'actual_qd_5']].to_numpy()
p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
aa = demo[['actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()
currents = demo[['actual_current_0', 'actual_current_1', 'actual_current_2', 'actual_current_3', 'actual_current_4', 'actual_current_5']].to_numpy()
target_moments = demo[['target_moment_0', 'target_moment_1', 'target_moment_2', 'target_moment_3', 'target_moment_4', 'target_moment_5']].to_numpy()
target_currents = demo[['target_current_0', 'target_current_1', 'target_current_2', 'target_current_3', 'target_current_4', 'target_current_5']].to_numpy()
force = demo[['actual_TCP_force_0', 'actual_TCP_force_1', 'actual_TCP_force_2']].to_numpy()

#make subfigure for q

pose = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2', 'actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()

plt.subplot(2, 3, 1)




