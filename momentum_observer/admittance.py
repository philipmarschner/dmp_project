import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
import time
import quaternion
from spatialmath import UnitQuaternion

class Admitance:
    def __init__(self,robot,kp,ko,Tc,dt = None):
        self.kdo = None
        self.kdp = None
        self.Ad = None
        self.Sp = None
        self.robot = robot
        self.kp = np.eye(3)*kp
        self.ko = np.eye(3)*ko
        self.M = np.eye(3)
        self.deltaqPos = np.zeros((3,1))
        self.deltaqdPos = np.zeros((3,1))
        self.deltaqddPos = np.zeros((3,1))
        self.deltaW = np.zeros((3,1))
        self.deltaWd = np.zeros((3,1))
        self.deltaQuat = np.array([1,0,0,0]).reshape(4,1)

        self.t = 0
        if not dt:
            self.dt = 0.002

     
        self.Tc = Tc

        self.calcSp()

        self.calcAd()
        self.calcKdPosition()
        self.calcKdOrientation()

    def calcKdPosition(self):

        temp = np.eye(3)
        for i in range (3):
            temp[i,i] = 2*np.sqrt(self.M[i,i]*self.kp[i,i])
        self.kdp = temp



    def calcKdOrientation(self):
        temp = np.eye(3)
        for i in range (3):
            temp[i, i] = 2*np.sqrt(self.M[i, i]*self.ko[i, i])

        self.kdo = temp

    def skew(self, v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    def calcE(self, n, e):
        return n*np.eye(3)-self.skew(e)
    
    def quatExp(self,r):
        n = np.cos(np.linalg.norm(r))

        if(np.linalg.norm(r) < 0.00000001):
            return UnitQuaternion([1,0,0,0])
        else:

            e = r/np.linalg.norm(r)*np.sin(np.linalg.norm(r))
            e = e.reshape(3,)
            test = 1
        return UnitQuaternion([n,e[0],e[1],e[2]])


    def calcSp(self):
        self.Sp = self.skew(self.Tc[0:3,3])

    def calcAd(self):
        Rc = self.Tc[0:3, 0:3]
        temp = np.concatenate((Rc, np.zeros((3, 3))), axis=1)
        temp2 = np.concatenate((self.Sp@Rc, Rc), axis=1)
        self.Ad = np.concatenate((temp, temp2), axis=0)

    def transformWrench(self, wrench):

        return np.transpose(self.Ad)@wrench

    def getForce(self, wrench):
        return self.transformWrench(wrench)[3:6]

    def getTorque(self, wrench):
        return self.transformWrench(wrench)[0:3]

    def calcPosqdd(self, wrench):

        self.deltaqddPos = np.linalg.inv(self.M)@(self.getForce(wrench)-self.kp@self.deltaqPos-self.kdp@self.deltaqdPos)
        self.deltaqPos += self.deltaqdPos*self.dt
        self.deltaqdPos += self.deltaqddPos*self.dt
        
        
        
        self.t += self.dt

    def quatIntegrate(self,w):
        r =self.dt/2*w.T
        #r = np.insert(r,0,0) 

        rquat = self.quatExp(r)


        temp = rquat*quaternion.from_float_array(self.deltaQuat.T)

        return  quaternion.as_float_array(temp).reshape(4,1)
    
    def getKQuat(self,q):
        return 2*self.calcE(q[0,0],q[1:4,0]).T@self.ko

       

        

    def calcOriqdd(self, wrench):
        self.deltaWd = np.linalg.inv(self.M)@(self.getTorque(wrench)-self.kdo@self.deltaW-self.getKQuat(self.deltaQuat)@self.deltaQuat[1:4])
        self.deltaW = self.deltaWd*self.dt
        self.deltaQuat = self.quatIntegrate(self.deltaW)
        
        
        

        return

    def getDeltaqPos(self):
        return self.deltaqPos
    
    def getDeltaqdPos(self):
        return self.deltaqdPos
    
    def getDeltaqddPos(self):
        return self.deltaqddPos
    
    def getDeltaQuat(self):
        return self.deltaQuat
    
    def getDeltaW(self):
        return self.deltaW
    

    def step(self, wrench):
        self.calcPosqdd(wrench)
        self.calcOriqdd(wrench)
        return self.deltaqPos,self.deltaQuat








