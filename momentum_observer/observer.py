#!/usr/bin/env python3
import numpy as np
import copy
import ur_robot
np.set_printoptions(precision=6, suppress=True)

import roboticstoolbox as rtb
import spatialmath as sm



#in joint space
class Observer:
    def __init__(self,Ko,model):
        self.Ko = Ko
        self.model = model
        self.integral = np.zeros((6,))
        self.oldR = np.zeros((6,))
        self.dt = 0.002
        #self.fc = np.array([12.54,13.27,4.99,2.0,2.69,2.3])
        #self.fc = np.array([14.0,7.0,10.0,2.0,3.0,2.0])
        self.fc = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        self.robot = ur_robot.URRobot()
        

        #grav = robot.gravity(q).reshape((6,1))
        #print("grav\n", grav)
#
        #jac = robot.jacobian(q)
        #print("Jacobian\n", jac)
#
        #jacDot = robot.jacobianDot(q, dq)
        #print("Jacobian dot\n", jacDot)
#
        #inertia = robot.inertia(q)
        #print("Inertia matrix\n", inertia)
#
        #coriolis = robot.coriolis(q, dq)
        #print("Coriolis matrix\n", coriolis)
#
        #print("Velocity product\n", coriolis @ dq)
#
        #tau = inertia @ ddq + coriolis @ dq + grav
        #print("Joint torque\n", tau)

        
    
    def calcR(self,tau,ds,q,qd):

        

        #self.integral += (tau + np.transpose(self.model.coriolis(q,qd))@qd-self.model.gravload(q) +self.oldR)*ds

        self.integral += (tau + np.transpose(self.robot.coriolis(q,qd))@qd-self.robot.gravity(q) +self.oldR)*ds

        if(qd[0] > 0.01):
            pass

        r = self.Ko*(self.robot.inertia(q)@qd-self.integral)
        self.oldR = copy.deepcopy(r)



        return r
    
    
    
    def reset(self):
        self.integral = np.zeros((6,1))
        self.oldR = np.zeros((6,1))