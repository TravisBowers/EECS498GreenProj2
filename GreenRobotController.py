from RobotArm import*
from GreenRobotDriver import*
import numpy as np
import ckbot.logical as L
import time

class GreenController(JoyApp):
    def __init__(self, *args, **kw):
    	self.driver = GreenDriver();
    	self.savedPoses = np.empty(6,1)



    #This function will take any point on the plane of the paper and 
    #convert the point to a point in the three dimensional robot frame
    def paperToWorld(self, paperX, paperY):
    	'''
    	put the math for Baddu's transform here!!!
    	'''

    	x=
    	y=
    	z=
    	self.driver.driveToPoint(i)
    	p = np.matrix([x,y,z]).T
    	return p


   	def drawRect(self):

   	def populatePoses(self):
   		#Set all servos slack
   		#manually move to point
   		#record the servos' positions
   		#convert positions to angles
   		#set angle values in model
   		#save arm state using driver.model.getPosValues()
