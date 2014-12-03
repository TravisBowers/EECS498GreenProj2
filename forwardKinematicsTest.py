from RobotArm import*
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	
	arm = GreenRobotArm();
	print("Arm has been initialized!!!")
	'''
	arm.setQ1(0)
	arm.setQ5(np.pi/2)
	arm.setQ4(0)
	arm.setQ3(0)
	'''
	armJac = arm.getJacobianNumeric();
	print('printing arm jabobian matrix')
	print(armJac)

	d = np.matrix([7,4,1]).T
	print('moving end effector to point ')

	print(d)
	arm.goToPoint(d)
	print('current end effector position')
	print(arm.getPointsColumn())
	e = d-arm.getPointsColumn()
	print('printing current error')
	print(e)
	#arm.plot3dArm()
	#fig1 = plt.figure()
	#fig1.set_visible(1)

	#arm.setQ5(np.pi/2)
	#arm.plot3dArm()
	#fig1 = plt.figure()
	#fig1.set_visible(1)


	'''	
	#arm.plotXY()

	#plt.figure()
	#arm.plotYZ()

	#plt.figure()
	#arm.plotXZ()

	i=0
	while(i<30):
	    fig1.set_visible(1)
	    
	    #Update the state of the rigid arm so that it's one step closer to the desired arm state
	    arm.setQ1(.05*i)
	    arm.setQ2(.1*i)
	    arm.setQ5(.04*i)
	    #display the updated arm state on the figure
	    #arm.plotXY()
	    
	    arm.plotXZ()
	    #arm.plotYZ()
	    plt.draw()
	    i=i+1

'''
