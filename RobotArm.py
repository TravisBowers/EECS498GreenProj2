
import time
import scipy 
import numpy as np
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import scipy as scipy
import sympy as sym
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


#Robot Arm Model
#This file will contain an object oriented implementation of a drawing robot class


class GreenRobotArm():
    def __init__(self, *args, **kw):
        
        '''
        might we want to initialize these to be pointing in the positive y direction?
        '''
        print('initializing arm')
        self.L1 = 5#150
        self.L2 = 5#240
        self.L3 = 5#300
        self.L4= 2#80
        
        #Joint angles All need to be with respect to something


        # 
        self.q1= 0
        self.q1Max = np.pi*3; 
        self.q1Min = -np.pi*3;

        self.q2 = 0
        self.q2Max = np.pi/2; 
        self.q2Min = -np.pi/2;
        
        self.q3 = 0
        self.q3Max = np.pi/2; 
        self.q3Min = -np.pi/2;
        
        self.q4 = 0#+np.pi/4
        self.q4Max = np.pi/2; 
        self.q4Min = -np.pi/2;
        
        self.q5 = 0
        self.q5Max = np.pi/2; 
        self.q5Min = -np.pi/2;

        
        #self.workMatrix = self.generateInverseTable();

       #A whole bunch of math based on the lengths and current angles of 
       #The robot's joint angles and rigid body lengths. 
    def setQ1(self, inputQ1):
        
        if(inputQ1> self.q1Max):
            print('attempted to exceed maximum threshold for q1: '+ str(self.q1Max))
            print('q1 will be set to:' + str(self.q1Max))
            newQ1 = self.q1Max
        elif(inputQ1<self.q1Min):
            newQ1 = self.q1Max
        else:
            print('attempted to exceed mainimum threshold for q1: '+ str(self.q5Min))
            print('q1 will be set to:' + str(self.q1Min))
            newQ1 = inputQ1

        self.q1 = inputQ1
        #print('self.q1 has been changed to'+str(self.q1))
        

    def setQ2(self, inputQ2):
        if(inputQ2> self.q2Max):
            print('attempted to exceed maximum threshold for q2: '+ str(self.q2Max))
            print('q2 will be set to:' + str(self.q2Max))
            newQ2 = self.q2Max
        elif(inputQ2<self.q2Min):
            newQ2 = self.q2Max
        else:
            print('attempted to exceed mainimum threshold for q2: '+ str(self.q2Min))
            print('q2 will be set to:' + str(self.q2Min))
            newQ2 = inputQ2

        self.q2 = inputQ2
    
    def setQ3(self, inputQ3):
        if(inputQ3> self.q3Max):
            print('attempted to exceed maximum threshold for q3: '+ str(self.q3Max))
            print('q3 will be set to:' + str(self.q3Max))
            newQ3 = self.q3Max
        elif(inputQ3<self.q3Min):
            newQ3 = self.q3Max
        else:
            print('attempted to exceed mainimum threshold for q3: '+ str(self.q3Min))
            print('q3 will be set to:' + str(self.q3Min))
            newQ3 = inputQ3

        self.q3 = inputQ3
    
    def setQ4(self, inputQ4):
        if(inputQ4> self.q4Max):
            print('attempted to exceed maximum threshold for q4: '+ str(self.q4Max))
            print('q4 will be set to:' + str(self.q4Max))
            newQ4 = self.q4Max
        elif(inputQ4<self.q4Min):
            newQ4 = self.q4Max
        else:
            print('attempted to exceed mainimum threshold for q4: '+ str(self.q4Min))
            print('q4 will be set to:' + str(self.q4Min))
            newQ4 = inputQ4

        self.q4 = inputQ4
    
    def setQ5(self, inputQ5):
        if(inputQ5> self.q5Max):
            print('attempted to exceed maximum threshold for q5: '+ str(self.q5Max))
            print('q5 will be set to:' + str(self.q5Max))
            newQ5 = self.q5Max
        elif(inputQ5<self.q5Min):
            newQ5 = self.q5Max
        else:
            print('attempted to exceed mainimum threshold for q5: '+ str(self.q5Min))
            print('q5 will be set to:' + str(self.q5Min))
            newQ5 = inputQ5

        self.q5 = inputQ5

    '''
    The function below will return a list of 5 4x4 matrices that describe the position and orientations of the 
    robot's 5 critical points. These points include the origin, the tool point, and the three intermediate joint points
    '''
    def forwardKinematics(self):
    	#print('computing forwardKinematics')


        #The default position of the tool point 
        gst_0= np.matrix([[1, 0, 0, 0],[0, 1, 0, self.L2+self.L3+self.L4],[0, 0, 1, self.L1],[0, 0, 0, 1]]);
        #The default position of the joint located 1 length behind the tool (the point between L3 and L4)
        gs3_0= np.matrix([[1, 0, 0, 0],[0, 1, 0, self.L2+self.L3],[0, 0, 1, self.L1],[0, 0, 0, 1]]);
        #The default position of the joint located 2 lengths behind the tool (The point between L2 and L3)
        gs2_0= np.matrix([[1, 0, 0, 0],[0, 1, 0, self.L2],[0, 0, 1, self.L1],[0, 0, 0, 1]]);
        #The default position of the joint located 3 lengths behind the tool (The point between L1 and L2 It shouldn't really move in space, but it will rotate)
        gs1_0= np.matrix([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, self.L1],[0, 0, 0, 1]]);
        
        #The default position of the origin point of the robot(the point at the lower end of L1)
        gs0_0= np.matrix([[1, 0, 0, 0],[0, 1, 0, self.L2+self.L3+self.L4],[0, 0, 1, self.L1],[0, 0, 0, 1]]);

    	w1= np.matrix([0,0,1])

    	w2= np.matrix([-1,0,0])

    	w3= w2

    	w4= np.matrix([0,1,0])

    	w5= w2

    	q1= np.matrix([0,0,self.L1])

    	q2= np.matrix([0,0,self.L1])

    	q3= np.matrix([0, self.L2, self.L1])

    	q4= np.matrix([0, self.L2+self.L3, self.L1])

    	q5 = q4


        v1= np.matrix(np.cross(-w1, q1)).T
    	v2= np.matrix(np.cross(-w2, q2)).T
    	v3= np.matrix(np.cross(-w3, q3)).T
    	v4= np.matrix(np.cross(-w4, q4)).T
    	v5= np.matrix(np.cross(-w5, q5)).T

    	xi1 = np.concatenate((v1, w1.T),axis=0)
    	xi2 = np.concatenate((v2, w2.T),axis=0)
    	xi3 = np.concatenate((v3, w3.T),axis=0)
    	xi4 = np.concatenate((v4, w4.T),axis=0)
    	xi5 = np.concatenate((v5, w5.T),axis=0)

    	e_wq1= np.matrix([[np.cos(self.q1),-np.sin(self.q1),0],[np.sin(self.q1), np.cos(self.q1), 0],[0,0,1]])

    	e_wq2= np.matrix([[1,0,0],[0,np.cos(self.q2),np.sin(self.q2)],[0,-np.sin(self.q2),np.cos(self.q2)]])

    	e_wq3= np.matrix([[1,0,0],[0,np.cos(self.q3),np.sin(self.q3)],[0,-np.sin(self.q3),np.cos(self.q3)]])

    	e_wq4 = np.matrix([[np.cos(self.q4),0,np.sin(self.q4)],[0,1,0],[-np.sin(self.q4),0,np.cos(self.q4)]])

    	e_wq5= np.matrix([[1,0,0],[0,np.cos(self.q5),np.sin(self.q5)],[0,-np.sin(self.q5),np.cos(self.q5)]])

    	K1= (np.identity(3)-e_wq1)*((np.cross(w1,v1.T).T));
    	K2= (np.identity(3)-e_wq2)*((np.cross(w2,v2.T).T));
    	K3= (np.identity(3)-e_wq3)*((np.cross(w3,v3.T).T));
    	K4= (np.identity(3)-e_wq4)*((np.cross(w4,v4.T).T));
    	K5= (np.identity(3)-e_wq5)*((np.cross(w5,v5.T).T));

    	dummy=np.matrix('0 0 0 1');
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


        #The location of the tool given the robot arm's current joint angles
    	#This is a 4X4 matrix, and the top three values in the rightmost column describe it's cartesian coordinates wrt the to origin
     
        gst_q=(e_xiq1*e_xiq2*e_xiq3*e_xiq4*e_xiq5)*gst_0
        

        gs3_q=(e_xiq1*e_xiq2*e_xiq3*e_xiq4)*gs3_0
        
        gs2_q=(e_xiq1*e_xiq2*e_xiq3)*gs2_0
    	
        gs1_q=(e_xiq1*e_xiq2)*gs1_0

        gs0_q=(e_xiq1)*gs0_0

        #print(gst_q)

        return [gs0_q, gs1_q, gs2_q, gs3_q, gst_q]



    def getArmAngles(self):
        return np.matrix([self.q1, self.q2, self.q3, self.q4, self.q5]).T
    
    def setArmAngles(self, angles):
        self.setQ1(angles[0])
        self.setQ2(angles[1])
        self.setQ3(angles[2])
        self.setQ4(angles[3])
        self.setQ5(angles[4])
    
    #This function will comput the forward kinematics of the robot and returns the 
    #position matrix of the robot's current configuration.
    def getEffectorMatrix(self):
        matrices = self.forwardKinematics();

        return matrices[0];
    

    #This function will generate a 3 dimensional plot of the arm's
    #current state
    def getPlotPoints(self):
        armPoints = self.forwardKinematics()

        
        #print('printing matrices :)')

        p0Matrix = armPoints[0]
        #print(p0Matrix)
        #print("")
        p1Matrix = armPoints[1]
        #print(p1Matrix)
        #print("")
        p2Matrix = armPoints[2]
        #print(p2Matrix)
        #print("")
        p3Matrix = armPoints[3]
        #print(p3Matrix)
        #print("")
        p4Matrix = armPoints[4]
        #print(p4Matrix)
        

        p0x= 0#p0Matrix[0,3]
        p0y= 0#p0Matrix[1,3]
        p0z= 0#p0Matrix[2,3]

        p1x= p1Matrix[0,3]
        p1y= p1Matrix[1,3]
        p1z= p1Matrix[2,3]

        p2x= p2Matrix[0,3]
        p2y= p2Matrix[1,3]
        p2z= p2Matrix[2,3]

        p3x= p3Matrix[0,3]
        p3y= p3Matrix[1,3]
        p3z= p3Matrix[2,3]

        p4x= p4Matrix[0,3]
        p4y= p4Matrix[1,3]
        p4z= p4Matrix[2,3]


        #print('q1 is now '+str(self.q1))
        x=[p0x,p1x,p2x,p3x,p4x]
        #print(x)

        y=[p0y,p1y,p2y,p3y,p4y]

        z=[p0z,p1z,p2z,p3z,p4z]


        return [x,y,z]
        #plt.plot(x,z)
        #plt.plot(y,z)

    #This function will return a unit column vector in the direction of the end effector
    def computeMarkerOrientation(self):
        armPoints = self.getPlotPoints()

        xs = armPoints[0]
        ys = armPoints[1]
        zs = armPoints[2]

        p5 = np.matrix([xs[4], ys[4], zs[4]]).T
        p4 = np.matrix([xs[3], ys[3], zs[3]]).T

        r = p5-p4
        rMag = np.linalg.norm(r)

        return (r/rMag)


    def getPointsColumn(self):
        armPoints = self.getPlotPoints()
        xs=armPoints[0]
        effectorX = xs[4]
        
        ys=armPoints[1]
        effectorY = ys[4]

        zs=armPoints[2]
        effectorZ = zs[4]

        pointsColumn= np.matrix([effectorX,effectorY,effectorZ]).T


        #print('the pointsColumn is of shape'+str(pointsColumn.shape))
        return pointsColumn

    def getPosValues(self):

        effectorCoords = self.getPointsColumn();
        effectorDir = self.computeMarkerOrientation();

        return np.concatenate((effectorCoords,effectorDir))




    def getJacobianNumeric2(self):
        dTheta = .01

        th0 = self.getPosValues()
        #print(th0)
        self.setQ1(self.q1+dTheta)
        thq1 = self.getPosValues()
        #print('the type of thq1 is '+str(type(thq1)))
        #print(thq1)
        #print('the shape of thq1 is '+str(thq1.shape))
        self.setQ1(self.q1-dTheta)

        j1=(thq1-th0)/dTheta
        #print('the shape of j1 is '+str(j1.shape))

        self.setQ2(self.q2+dTheta)
        thq2 = self.getPosValues()
        self.setQ2(self.q2-dTheta)
        j2=(thq2-th0)/dTheta

        self.setQ3(self.q3+dTheta)
        thq3 = self.getPosValues()
        self.setQ3(self.q3-dTheta)
        j3=(thq3-th0)/dTheta

        self.setQ4(self.q4+dTheta)
        thq4 = self.getPosValues()
        self.setQ4(self.q4-dTheta)
        j4=(thq4-th0)/dTheta

        self.setQ5(self.q5+dTheta)
        thq5 = self.getPosValues()
        self.setQ5(self.q5-dTheta)
        j5=(thq5-th0)/dTheta

        jac = np.concatenate((j1,j2,j3,j4,j5),axis=1)

        #print('the jacobian is of shape '+str(jac.shape))

        return jac;        


    
    #This function will calculate a jacobian matrix of the robot arm
    def getJacobianNumeric(self):
        dTheta = .01

        th0 = self.getPointsColumn()
        #print(th0)
        self.setQ1(self.q1+dTheta)
        thq1 = self.getPointsColumn()
        #print('the type of thq1 is '+str(type(thq1)))
        #print(thq1)
        #print('the shape of thq1 is '+str(thq1.shape))
        self.setQ1(self.q1-dTheta)

        j1=(thq1-th0)/dTheta
        #print('the shape of j1 is '+str(j1.shape))

        self.setQ2(self.q2+dTheta)
        thq2 = self.getPointsColumn()
        self.setQ2(self.q2-dTheta)
        j2=(thq2-th0)/dTheta

        self.setQ3(self.q3+dTheta)
        thq3 = self.getPointsColumn()
        self.setQ3(self.q3-dTheta)
        j3=(thq3-th0)/dTheta

        self.setQ4(self.q4+dTheta)
        thq4 = self.getPointsColumn()
        self.setQ4(self.q4-dTheta)
        j4=(thq4-th0)/dTheta

        self.setQ5(self.q5+dTheta)
        thq5 = self.getPointsColumn()
        self.setQ5(self.q5-dTheta)
        j5=(thq5-th0)/dTheta

        jac = np.concatenate((j1,j2,j3,j4,j5),axis=1)

        #print('the jacobian is of shape '+str(jac.shape))

        return jac;

    #p should be a column matrix of shape 3x1 
    def goToPoint(self,p):
        errorMag=2;
        threshold = .01
        maxIterations = 100
        count =0
        while (errorMag >threshold):
            l_d=p;#%[0;400;35]; #%end point (desired location)
            l_c=self.getPointsColumn();# current coordinates of the robot arm 
            e=l_d-l_c; #column of values representing the error between current and desired points
            Jac= self.getJacobianNumeric(); # jacobian matrix 
            Jac_inv=np.linalg.pinv(Jac); # inverse jacobian matrix
            angles = self.getArmAngles()
            #print('the shape of e')
            #print(e.shape)
            #print('the shape of Jac')
            #print(Jac.shape)
            #print(self.getArmAngles())
            th_d= angles+(np.dot(Jac_inv,e)) #desired joint angles

            self.setQ1(th_d.item(0)%(np.pi*2))
            self.setQ2(th_d.item(1)%(np.pi*2))
            self.setQ3(th_d.item(2)%(np.pi*2))
            self.setQ4(th_d.item(3)%(np.pi*2))
            self.setQ5(th_d.item(4)%(np.pi*2))
                #subs(Tr); #%to check for intermediate coordinate points
            newPoints = self.getPointsColumn()
            newError = l_d-newPoints
            errorMag=np.linalg.norm(newError); #% calculating the norm for running the loop
            '''
            there may be a case in which the arm will attempt to travel to a point that is 
            not reachable by the arm. If this is the case, the loop will go on forever trying to get to the point
            the code below will break the loop after a certain iteration threshold
            '''
            if(count>=maxIterations):
                print('the loop has iterated '+ str(maxIterations)+' times and the point has not been reached. ')
                print('navigation to point:')
                print(p)
                print('has failed')
                break
            
            count = count+1
            #print('the while loop has performed '+str(count)+ ' iterations')
        return np.matrix([self.q1,self.q2,self.q3,self.q4,self.q5]).T

    
    def goToPoint2(self, p):
        errorMag=2;
        threshold = .01
        maxIterations = 100
        count =0
        while (errorMag >threshold):
            l_d=p;#%[0;400;35]; #%end point (desired location and orientation)
            l_c=self.getPosValues();# current coordinates of the robot arm 
            e=l_d-l_c; #column of values representing the error between current and desired points
            Jac= self.getJacobianNumeric2(); # jacobian matrix 
            Jac_inv=np.linalg.pinv(Jac); # inverse jacobian matrix
            angles = self.getArmAngles()
            #print('the shape of e')
            #print(e.shape)
            #print('the shape of Jac')
            #print(Jac.shape)
            #print(self.getArmAngles())
            th_d= angles+(np.dot(Jac_inv,e)) #desired joint angles

            self.setQ1(th_d.item(0)%(np.pi*2))
            self.setQ2(th_d.item(1)%(np.pi*2))
            self.setQ3(th_d.item(2)%(np.pi*2))
            self.setQ4(th_d.item(3)%(np.pi*2))
            self.setQ5(th_d.item(4)%(np.pi*2))
                #subs(Tr); #%to check for intermediate coordinate points
            newPoints = self.getPosValues()
            newError = l_d-newPoints
            errorMag=np.linalg.norm(newError); #% calculating the norm for running the loop
            '''
            there may be a case in which the arm will attempt to travel to a point that is 
            not reachable by the arm. If this is the case, the loop will go on forever trying to get to the point
            the code below will break the loop after a certain iteration threshold
            '''
            if(count>=maxIterations):
                print('the loop has iterated '+ str(maxIterations)+' times and the point has not been reached. ')
                print('navigation to point:')
                print(p)
                print('has failed')
                break
            
            count = count+1
            #print('the while loop has performed '+str(count)+ ' iterations')
        return np.matrix([self.q1,self.q2,self.q3,self.q4,self.q5]).T


    def plotXY(self):
        points = self.getPlotPoints()
        plt.plot(points[0],points[1])

    def plotXZ(self):
        points = self.getPlotPoints()
        plt.plot(points[0],points[2])

    def plotYZ(self):
        points = self.getPlotPoints()
        plt.plot(points[1],points[2])


    def plotProjections(self):

        self.plotXY()
        self.plotXZ()
        self.plotYZ()

    def plot3dArm(self):
        points = self.getPlotPoints()

        fig = plt.figure()
        fig.set_visible(1)
        ax = fig.add_subplot(111, projection='3d')
        

        p=ax.plot_wireframe(points[0],points[1],points[2])

    '''
    This function should generate an enormous data structure that contains all of the robot arm's 
    possible end effector configurations
    '''
    def generateInverseTable(self):
        t1=0
        t2=0
        t3=0
        t4=0
        t5=0
        startTime = time.clock()
        table = np.empty((1,11))
        #print('table Shape')
        #print(table.shape)
        numAngles = 1#180*100

        while(self.q1<=np.pi/2):
            self.setQ1(-np.pi/2+((np.pi*t1)/numAngles))
            print('iterating through q1')
            print('makin a table. q1 is')

            print(self.q1)
            current = time.clock()
            print('time Elapsed: ')
            print(current-startTime)
            t2=0
            t1 = t1+1
            self.setQ2(0)
            while(self.q2<=np.pi/2):
                print('iterating through q2')
                self.setQ2(-np.pi/2+((np.pi*t2)/numAngles))
                t2=t2+1
                t3=0
                self.setQ3(0)
                while(self.q3<=np.pi/2):
                    print('iterating through q3')
                    self.setQ3(-np.pi/2+((np.pi*t3)/numAngles))
                    t3=t3+1
                    t4=0
                    self.setQ4(0)
                    while(self.q4<=np.pi/2):
                        print('iterating through q4')
                        self.setQ4(-np.pi/2+((np.pi*t4)/numAngles))
                        t4=t4+1
                        t5=0
                        self.setQ5(0)
                        while(self.q5<=np.pi/2):
                            print('iterating through q5')
                            self.setQ5(-np.pi/2+((np.pi*t5)/numAngles))
                            t5=t5+1
                            armPoints = self.getPlotPoints()
                            armxs=armPoints[0]
                            armys=armPoints[1]
                            armzs= armPoints[2]

                            xdir = (armxs[4]-armxs[3])/self.L4
                            ydir = (armys[4]-armys[3])/self.L4
                            zdir = (armzs[4]-armzs[3])/self.L4

                            newVector = np.array([[self.q1, self.q2, self.q3, self.q4, self.q5, armxs[4],armys[4],armzs[4],xdir,ydir,zdir]])
                            #print('newVector shape')
                            #print(newVector.shape)
                            table =np.concatenate((table, newVector), axis=0)
                            print('still chuggin be patient')
                            print(table.shape)

        self.setQ1(0)
        self.setQ2(0)
        self.setQ3(0)
        self.setQ4(0)
        self.setQ5(0)
        print('the table has been initialized its shape is below')
        print(table.shape)
        return table






        
        
        





    '''
    This function will perform inverse kinematics of the robot's arm and will set the arm's angles in a configuration
    such that the robot's marker tip is located at the inputted x,y,z coordinates and at the inputted direction vector
    '''
    #def setArm(self,Xin, Yin, Zin, Approach):