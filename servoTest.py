import sympy as sym
from RobotArm import*
from GreenRobotDriver import*
import numpy as np


if __name__ == '__main__':

	model = GreenRobotArm()
	print('model initialized')
	arm = GreenDriver()

	arm.goToAngles(np.pi/5, np.pi/5)

	print('done')