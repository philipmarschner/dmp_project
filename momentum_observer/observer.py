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
        #self.fc = np.array([12.54,13.27,4.99,2.0,2.69,2.3])
        self.fv = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        #self.fv = np.array([0.055,0.064,0.050,0.114,0.107,0.015])
        #self.fc = np.array([14.0,7.0,10.0,2.0,3.0,2.0])
        self.fc = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        
    
    def calcR(self,tau,ds,q,qd):

        

        self.integral += (tau + np.transpose(self.model.coriolis(q,qd))@qd-self.model.gravload(q) +self.oldR)*ds

        r = self.Ko*(self.model.inertia(q)@qd-self.integral)
        self.oldR = r



        return r
    
    
    
    def reset(self):
        self.integral = np.zeros((6,1))
        self.oldR = np.zeros((6,1))