import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
import time
import quaternion
#from livefilter import LiveLFilter
import scipy.signal

#from spatialmath import UnitQuaternion

class Admitance:
    def __init__(self,robot,kp,ko,Tc,kdp,kdo,dt = None):
        self.kdo = np.eye(3)*kdo
        self.kdp = np.eye(3)*kdp
        self.Ad = None
        self.Sp = None
        self.kp = np.eye(3)*kp
        self.ko = np.eye(3)*ko
        self.M = np.eye(3)
        self.deltaqPos = np.zeros((3,1))
        self.deltaqdPos = np.zeros((3,1))
        self.deltaqddPos = np.zeros((3,1))
        self.deltaW = np.zeros((3,1))
        self.deltaWd = np.zeros((3,1))
        self.deltaQuat = np.array([1,0,0,0]).reshape(4,1)
        b, a = scipy.signal.iirfilter(10, Wn=10, fs=500, btype="low", ftype="butter")
        #self.filterx = LiveLFilter(b,a)
        #self.filtery = LiveLFilter(b,a)
        #self.filterz = LiveLFilter(b,a)
        #self.filtertx = LiveLFilter(b,a)
        #self.filterty = LiveLFilter(b,a)
        #self.filtertz = LiveLFilter(b,a)

        self.lastWrench = np.zeros((1,6))

        self.t = 0
        if not dt:
            self.dt = 0.002
        else:
            self.dt = dt

     
        self.Tc = Tc

        self.calcSp()
        self.calcAd()
        #self.calcKdPosition()
        self.calcKdOrientation()

    def calcKdPosition(self):

        temp = np.eye(3)
        for i in range (3):
            temp[i,i] = 2*np.sqrt(self.M[i,i]*self.kp[i,i])
        self.kdp = temp

    def exponentialFilter(self,wrench):
        alpha = 0.8

        lastArray = np.array(self.lastWrench)
        wrenchArray = np.array(wrench)

        lastWrenchArray = alpha*lastArray + (1-alpha)*wrenchArray
        self.lastWrench = lastWrenchArray.tolist()


        if np.abs(self.lastWrench[0][0]) < 0.5:
            self.lastWrench[0][0] = 0
        if np.abs(self.lastWrench[0][1]) < 0.5:
            self.lastWrench[0][1] = 0
        if np.abs(self.lastWrench[0][2]) < 0.5:
            self.lastWrench[0][2] = 0

        return self.lastWrench



    def filter_wrench(self, wrench):
        temp_wrench = wrench
        temp_wrench[0] = self.filterx._process(wrench[0])
        temp_wrench[1] = self.filtery._process(wrench[1])
        temp_wrench[2] = self.filterz._process(wrench[2])
        temp_wrench[3] = self.filtertx._process(wrench[3])
        temp_wrench[4] = self.filterty._process(wrench[4])
        temp_wrench[5] = self.filtertz._process(wrench[5])


         #find largest element in force
        max_force_index = np.argmax(np.abs(temp_wrench[0:3]))


        if temp_wrench[max_force_index] < 0.5:
            temp_wrench = [0,0,0,0,0,0]



        return temp_wrench


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
        print("This function is depreciated, exitting...")
        exit()
        n = np.cos(np.linalg.norm(r))

        if(np.linalg.norm(r) < 0.00000001):
            return ([1,0,0,0])
        else:

            e = r/np.linalg.norm(r)*np.sin(np.linalg.norm(r))
            e = e.reshape(3,)
            test = 1
            
        return np.array([[n,e[0],e[1],e[2]]]).reshape(4,1) 


    def calcSp(self):
        self.Sp = self.skew(self.Tc[0:3,3])

    def calcAd(self):
        Rc = self.Tc[0:3, 0:3]
        temp = np.concatenate((Rc, np.zeros((3, 3))), axis=1)
        temp2 = np.concatenate((self.Sp@Rc, Rc), axis=1)
        self.Ad = np.concatenate((temp, temp2), axis=0)

    def transformWrench(self, wrench):
        #todo byt om på torque og force 
        wrench_flipped = wrench.copy()
        wrench_flipped[3:6], wrench_flipped[0:3] = wrench[0:3], wrench[3:6]
        #todo did above, test if it works

        return np.transpose(self.Ad)@wrench_flipped

    def getForce(self, wrench):

        #filteredWrench = self.filter_wrench(wrench)

        filteredWrench = self.exponentialFilter(wrench)[0]

       


        return np.array(filteredWrench)[0:3]

    def getTorque(self, wrench):
        return np.array(wrench)[3:6]

    def calcPosqdd(self, wrench):

        self.deltaqddPos = np.linalg.inv(self.M)@(self.getForce(wrench).reshape(3,1)-self.kp@self.deltaqPos-self.kdp@self.deltaqdPos)
        self.deltaqPos += self.deltaqdPos*self.dt
        self.deltaqdPos += self.deltaqddPos*self.dt
        self.t += self.dt

    def quatIntegrate(self,w):
        r =self.dt/2*w.T
       
        #rquat = self.quatExp(r)

        temp = quaternion.from_rotation_vector(r)
        rquat = np.exp(temp)

        temp = rquat*quaternion.from_float_array(self.deltaQuat.T.reshape(4,))

        #return  quaternion.as_float_array(temp).reshape(4,1)

        return temp
    
    def getKQuat(self,q):
        return 2*self.calcE(q[0,0],q[1:4,0]).T@self.ko

       

        

    def calcOriqdd(self, wrench):
        self.deltaWd = np.linalg.inv(self.M)@(self.getTorque(wrench).reshape(3,1)-self.kdo@self.deltaW-self.getKQuat(self.deltaQuat)@self.deltaQuat[1:4])
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
    
    def reset(self):
        self.deltaqPos = np.zeros((3,1))
        self.deltaqdPos = np.zeros((3,1))
        self.deltaqddPos = np.zeros((3,1))
        self.deltaW = np.zeros((3,1))
        self.deltaWd = np.zeros((3,1))
        self.deltaQuat = np.array([1,0,0,0]).reshape(4,1)







