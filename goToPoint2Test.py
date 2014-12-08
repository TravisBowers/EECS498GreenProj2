import sympy as sym
from RobotArm import*
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	arm = GreenRobotArm()

	print('printing current arm configuration')
	print(arm.getPosValues())

	d = np.matrix([7,0,4,0,0,1]).T

	print('beginning navigation to point: ')
	print(d)
	arm.goToPoint2(d)

	print('printing current arm configuration')
	print(arm.getPosValues())