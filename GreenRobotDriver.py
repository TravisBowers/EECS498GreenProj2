from RobotArm import*
import numpy as np
import ckbot.logical as L
import time

class GreenDriver():
    def __init__(self, *args, **kw):
    	self.model = GreenRobotArm();
    	self.c = L.Cluster()
    	#Here, the appropriate number of servos will be populated into the cluster
    	c.populate(5,{ 0x14 : 'j1',0x20:'j2',0x12:'j3',0x65:'j4',0x34:'j5'})



    #simple function that converts an angle in radians to an encoder position on a servo
    def angleToEncoder(self,angle):
    		
    		#converts the input angle to an encoder position in centidegrees
    		encoderPos = angle*(180/np.pi)*100;
    		return encoderPos


    
    def goToAngles(self, j1Theta,j2Theta,j3Theta,j4Theta,j5Theta,iterDelay,numInterps):
    	
    	j1Goal= self.angleToEncoder(j1Theta)
    	j2Goal= self.angleToEncoder(j2Theta)
    	j3Goal= self.angleToEncoder(j2Theta)
    	j4Goal= self.angleToEncoder(j2Theta)
    	j5Goal= self.angleToEncoder(j2Theta)

    	j1Start= self.c.at.j1.get_pos()
    	j2Start= self.c.at.j2.get_pos()
    	j3Start= self.c.at.j3.get_pos()
    	j4Start= self.c.at.j4.get_pos()
    	j5Start= self.c.at.j5.get_pos()

    	j1step = (j1Goal-j1Start)/numInterps
    	j2step = (j2Goal-j2Start)/numInterps
    	j3step = (j3Goal-j3Start)/numInterps
    	j4step = (j4Goal-j4Start)/numInterps
    	j5step = (j5Goal-j5Start)/numInterps

    	for i in range(numInterps+1):
    		

    	

