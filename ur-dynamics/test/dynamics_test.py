#!/usr/bin/env python3
import numpy as np
import time
np.set_printoptions(precision=6, suppress=True)

import ur_robot

q = np.array([[1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472]]).T
dq = np.array([[1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472]]).T
ddq = np.array([[1.0000,   1.0472,    1.0472, 1.0472, 1.0472, 1.0472]]).T

print(q)
print(dq)
print(ddq)

robot = ur_robot.URRobot()

times = 1000000
timing = np.zeros((times,1))
start_time = time.time()
for i in range(times):
    start = time.time()
    grav = robot.gravity(q).reshape((6,1))
    jac = robot.jacobian(q)
    jacDot = robot.jacobianDot(q, dq)
    inertia = robot.inertia(q)
    coriolis = robot.coriolis(q, dq)
    tau = inertia @ ddq + coriolis @ dq + grav
    end = time.time()
    timing[i] = end - start
end_time = time.time()


print(robot)
print("grav\n", grav)
print("Jacobian\n", jac)
print("Jacobian dot\n", jacDot)
print("Inertia matrix\n", inertia)
print("Coriolis matrix\n", coriolis)
print("Velocity product\n", coriolis @ dq)
print("Joint torque\n", tau)
print("Mean time elapsed: ", np.mean(timing)*1000, "ms")
print("Mean frequency: ", 1/np.mean(timing), "Hz")
print("Total time elapsed: ", end_time - start_time, "s")