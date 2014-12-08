from RobotArm import*
import numpy as np
import ckbot.logical as L
import time

class GreenDriver():
    def __init__(self, *args, **kw):
    	self.model = GreenRobotArm();
    	self.c = L.Cluster()
    	#Here, the appropriate number of servos will be populated into the cluster
    	'''
    	WARNING!!!: STILL NEED TO DETERMINE THE ADDRESS OF THE FIFTH SERVO. RUN A SCRIPT ONCE ITS SET UP
    	'''
    	c.populate(5,{ 0x13 : 'j1',0x1E:'j2',0x20:'j3',0x08:'j4',0x:'j5'})


    #simple function that will convert an encoder position in centidegrees to a joint angle in radians
    def encoderToangle(self,encoderPos):
    	angle = (encoderPos/100)*(np.pi/180)
    	return angle;

    #simple function that converts an angle in radians to an encoder position on a servo
    
    def angleToEncoder(self,angle):
    		
    		#converts the input angle to an encoder position in centidegrees
    		encoderPos = angle*(180/np.pi)*100;
    		return encoderPos


    
    def goToAngles(self, j1Theta,j2Theta,j3Theta,j4Theta,j5Theta):
    	
    	numInterps = 10
    	iterDelay = .06 

    	#determine final encoder values
    	j1Goal= self.angleToEncoder(j1Theta)
    	j2Goal= self.angleToEncoder(j2Theta)
    	j3Goal= self.angleToEncoder(j2Theta)
    	j4Goal= self.angleToEncoder(j2Theta)
    	j5Goal= self.angleToEncoder(j2Theta)

    	#determinal inital encoder values
    	j1Start= self.c.at.j1.get_pos()
    	j2Start= self.c.at.j2.get_pos()
    	j3Start= self.c.at.j3.get_pos()
    	j4Start= self.c.at.j4.get_pos()
    	j5Start= self.c.at.j5.get_pos()

    	#calculate the step size of each servo
    	j1step = (j1Goal-j1Start)/numInterps
    	j2step = (j2Goal-j2Start)/numInterps
    	j3step = (j3Goal-j3Start)/numInterps
    	j4step = (j4Goal-j4Start)/numInterps
    	j5step = (j5Goal-j5Start)/numInterps

    	#send stepping position commands to each servo then wait for the servos to reach their desired setpoint
    	for i in range(1,numInterps+1):
    		self.c.at.j1.set_pos(j1Start+i*j1Step)
    		self.c.at.j2.set_pos(j2Start+i*j2Step)
    		self.c.at.j3.set_pos(j3Start+i*j3Step)
    		self.c.at.j4.set_pos(j4Start+i*j4Step)
    		self.c.at.j5.set_pos(j5Start+i*j5Step)
    		time.sleep(iterDelay)

    	
    '''
    This function should drive the robot to a certain desired setpoint. This is using the goToPoint2 function in the model
    This function will navigate the arm to a desired setpoint with a desired end effector orientation
    the point p must be in the form of a matrix with shape (6x1) i.e. p = np.matrix([xPos, yPos, zPos, xDir, yDir, Zdir]) 
    '''
    def driveToPoint(self, p):

    	angles = self.model.goToPoint2(p)
    	
    	j1Theta = angles.item(0)
    	j2Theta = angles.item(1)
    	j3Theta = angles.item(2)
    	j4Theta = angles.item(3)
    	j5Theta = angles.item(4)

    	self.goToAngles(j1Theta,j2Theta,j3Theta,j4Theta,j5Theta)

