from RobotArm import*

if __name__ == '__main__':
	arm = GreenRobotArm();
	someValue = arm.forwardKinematics();

	print(someValue)