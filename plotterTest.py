from RobotArm import*
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def tofloat(thetuple): return float(thetuple)

if __name__ == '__main__':

	

	fig = plt.figure()
	ax = fig.gca(projection='3d')
	angle = 5*pi/180
	x=[50,266*cos(angle)]
	y=[0,294]
	z=[0,294*sin(angle)]
	x, y = np.meshgrid(x,y)
	#surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, linewidth=0)
	ax.set_zlim(0, 10)
	plt.axis([0,10, 0,10])

	ourArm = GreenRobotArm()
	plt.show()
	
	for theindex in range(10):
		
		d = np.matrix([7*np.cos(theindex*.4),4*np.sin(theindex*.4),theindex,0,0,1]).T
		#print('destination point defined')
		#raw_input()
		#print('the raw input line ran')
		#time.sleep(.2)
		#print('navigating to point')
		ourArm.goToPoint2(d)
		#print('arm has navigated to current waypoint')
		theMatrix = ourArm.getPlotPoints()
		theMatrixReloaded = zip(theMatrix[0],theMatrix[1],theMatrix[2])
		thenum = [0,0,0,0,0]
		#print('entering first for loop')
		for i in range(5):
			thenum[i] = map (tofloat, theMatrixReloaded[i])
		#print thenum
		#print('entering second for loop')
		for i in range(5):
			thenum2 = thenum[i]
			ax.scatter(thenum2[0],thenum2[1],thenum2[2], color = (0.3600,0.6000,0.5000,1))
			if i < 4:
				thenum3 = thenum[i+1]
				ax.plot([thenum2[0],thenum3[0]],[thenum2[1],thenum3[1]],[thenum2[2],thenum3[2]])
				plt.draw()
				plt.show()
		theindex+=2