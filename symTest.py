import sympy as sym
from RobotArm import*
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	x= sym.Symbol('x')

	f = sym.cos(x)

	d = sym.diff(f,x)

	



	print(d)

	x= np.pi/2

	print(d.subs(x,pi).evalf() )

