
import time
import scipy 
import numpy as np
import matplotlib as plt
from mpl_toolkits.mplot3d import Axes3D


#Robot Arm Model
#This file will contain an object oriented implementation of a drawing robot class


class GreenRobotArm():
    def __init__(self, *args, **kw):
        
        '''
        might we want to initialize these to be pointing in the positive y direction?
        '''

        self.L1 = 5
        self. L2 = 5
        self.L3 =5
        self.L4= 5
        
        #Joint angles All need to be with respect to something


        # 
        self.q1= 0
        
        self.q2 = 0
        
        self.q3 = 0
        
        self.q4 = 0
        
        self.q5 = 0

       #A whole bunch of math based on the lengths and current angles of 
       #The robot's joint angles and rigid body lengths. 
    '''
    def getJointPoints(self):
    	x1= 0
    	y1= 0
    	z1= self.L1

    	p1 = [x1 y1 z1]

    	x2= self.L2*cos((180-self.q1)*pi/180)
    	y2=self.L2*sin((180-self.q1)*pi/180)
    	z2=self.L1 + self.L2*cos((self.q2)*pi/180)

    	p2 = [x2 y2 z2]

    	x3=
    	y3=
    	z3=

    	p3 = [x3 y3 z3]

    	x4=
    	y4=
    	z4=

    	p4= [x4 y4 z4]

    	return [p1 p2 p3 p4]
'''
    def forwardKinematics(self):
    	print("computing forwardKinematics")
    	gst_0= matrix('1, 0, 0, 0;0, 1, 0, (self.L2+self.L3+self.L4)];0, 0, 1, self.L1;0, 0, 0, 1');

    	w1= matrix([0,0,1])

    	w2= matrix([-1,0,0])

    	w3= w2

    	w4= matrix([0,1,0])

    	w5= w2

    	q1= matrix([0,0,self.L1])

    	q2= matrix([0,0,0,self.L1])

    	q3= matrix([0, self.L2, self.L1])

    	q4= matrix([0, self.L2+self.L3, self.L1])

    	q5 = q4

    	v1= matrix(np.cross(-w1, q1)).T
    	v2= matrix(np.cross(-w2, q2)).T
    	v3= matrix(np.cross(-w3, q3)).T
    	v4= matrix(np.cross(-w4, q4)).T
    	v5= matrix(np.cross(-w5, q5)).T

    	xi1 = np.concatenate((v1, w1.T),axis=0)
    	xi2 = np.concatenate((v2, w2.T),axis=0)
    	xi3 = np.concatenate((v3, w3.T),axis=0)
    	xi4 = np.concatenate((v4, w4.T),axis=0)
    	xi5 = np.concatenate((v5, w5.T),axis=0)

    	e_wq1= matrix([[cos(self.q1),-sin(self.q1),0],[sin(self.q1), cos(self.q1), 0],[0,0,1]])

    	e_wq2= matrix([[1,0,0],[0,cos(self.q2),sin(self.q2)],[0,-sin(self.q2),cos(self.q2)]])

    	e_wq3= matrix([[1,0,0],[0,cos(self.q3),sin(self.q3)],[0,-sin(self.q3),cos(self.q3)]])

    	e_wq4 = matrix([[cos(self.q4),0,sin(self.q4)],[0,1,0],[-sin(self.q4),0,cos(self.q4)]])

    	e_wq5= matrix([[1,0,0],[0,cos(self.q5),sin(self.q5)],[0,-sin(self.q5),cos(self.q5)]])

    	K1= (identity(3)-e_wq1)*((cross(w1,v1.T).T));
    	K2= (identity(3)-e_wq2)*((cross(w2,v2.T).T));
    	K3= (identity(3)-e_wq3)*((cross(w3,v3.T).T));
    	K4= (identity(3)-e_wq4)*((cross(w4,v4.T).T));
    	K5= (identity(3)-e_wq5)*((cross(w5,v5.T).T));

    	dummy=matrix('0 0 0 1');
    	e_q1=np.concatenate((e_wq1, K1),axis=1)
    	e_xiq1=np.concatenate((e_q1,dummy),axis=0)

    	e_q2=np.concatenate((e_wq2, K2),axis=1)
    	
    	e_xiq2=np.concatenate((e_q2,dummy),axis=0)

    	e_q3=np.concatenate((e_wq3, K3),axis=1)
    	e_xiq3=np.concatenate((e_q3,dummy),axis=0)

    	e_q4=np.concatenate((e_wq4, K4),axis=1)
    	e_xiq4=np.concatenate((e_q4,dummy),axis=0)

    	e_q5=np.concatenate((e_wq5, K5),axis=1)
    	e_xiq5=np.concatenate((e_q5,dummy),axis=0)


    	gst_q=(e_xiq1*e_xiq2*e_xiq3*e_xiq4*e_xiq5)*gst_0

    	return gst_q


    #This function will return the cartesian coordinates of
    #the tip of the robot's marker
    def getMarkerPoint(self):
    	points = getJointPoints()

    	return points[3]

    def getMarkerDirection(self):
    	
    	points = getJointPoints()

    	p3 = points[2]

    	p4 = points[3]

    	return (p4-p3)/1

    
    #This function will generate a 3 dimensional plot of the arm's
    #current state
    
    #def plotArm(self):


    #def setArm(self,Xin, Yin, Zin, Approach):