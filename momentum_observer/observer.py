import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm



#in joint space
class Observer:
    def __init__(self,Ko,model):
        self.Ko = Ko
        self.model = model
        self.integral = np.zeros((6,))
        self.oldR = np.zeros((6,))
        self.dt = 0.02

        
    
    def calcR(self,tau,ds):

        q = self.model.q
        qd = self.model.qd

        self.integral += (tau + np.transpose(self.model.coriolis(q,qd))@qd-self.model.gravload(q) +self.oldR)*ds

        r = self.Ko*(self.model.inertia(q)@qd-self.integral)
        self.oldR = r

        return r
    
    
    
    def reset(self):
        self.integral = np.zeros((6,1))
        self.oldR = np.zeros((6,1))