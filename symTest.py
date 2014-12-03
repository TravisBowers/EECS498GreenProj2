import sympy as sym
from RobotArm import*
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	
	#
	x= sym.Symbol('x')

	#define the expression f as a function of a symbolic variable
	f = sym.cos(x)


	#will compute the partial derivative of f wrt x
	d = sym.diff(f,x)


	



	print(d)

	
	#substitutes the value of 3 in for x in the expression d
	s= d.subs({x: 3.0}).evalf(21)


	print(s)

