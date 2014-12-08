from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import time
import numpy as np
from RobotArm import GreenRobotArm

fig = plt.figure()
ax = fig.gca(projection='3d')

angle = 5*pi/180
x=[50,266*cos(angle)]
y=[0,294]
z=[0,294*sin(angle)]
#p1 = [2+(100*cos(angle),100,50*sin(angle)]
#p2 = [2+(150*cos(angle)),150,100*sin(angle)]
#pdiff = p2 - p1
#f = open('points.txt','w')
#for w in range(10)
	#f.write(p1+pdiff/10\n)
#x = [0,216,216,0]
#y = [0,0,294*sin(angle),sin(angle)*294]
#z = [0,0,cos(angle)*294,cos(angle)*294]


x, y = np.meshgrid(x,y)
surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, linewidth=0)
ax.set_zlim(0, 300)
plt.axis([0,300, 0,300])


plt.show()

i=0
the_file3=[0,0,0,0,0]
with open('output2.txt', 'r') as the_file2:
	for line in the_file2:

		the_file3[i] = line.split()
		i+=1
print the_file3
#print the_file3
the_file4=zip(the_file3[0],the_file3[1],the_file3[2],the_file3[3],the_file3[4])
print the_file4
print 'this is the_file4'
#print the_file4[0]
the_file5=the_file4[0]
def tofloat(thetuple): return float(thetuple)
the_file6 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
for i in range(20):
	the_file6[i] = map (tofloat, the_file4[i])
#print the_file6


print '\n'
print the_file6
print 'this is the file6'
theindex=10
while(theindex<20):
	raw_input()
	time.sleep(2)
	firstPos = the_file6[theindex]
	ourArm = GreenRobotArm()
	ourArm.setQ1(firstPos[0])
	ourArm.setQ2(firstPos[1])
	ourArm.setQ3(firstPos[2])
	ourArm.setQ4(firstPos[3])
	ourArm.setQ5(firstPos[4])
	theMatrix = ourArm.getPlotPoints()
	theMatrixReloaded = zip(theMatrix[0],theMatrix[1],theMatrix[2])
	thenum = [0,0,0,0,0]
	for i in range(5):
		thenum[i] = map (tofloat, theMatrixReloaded[i])
	#print thenum

	for i in range(5):
		thenum2 = thenum[i]
		ax.scatter(thenum2[0],thenum2[1],thenum2[2], color = (0.3600,0.6000,0.5000,1))
		if i < 4:
			thenum3 = thenum[i+1]
			ax.plot([thenum2[0],thenum3[0]],[thenum2[1],thenum3[1]],[thenum2[2],thenum3[2]])
			plt.draw()
			plt.show()
	theindex+=2




"""
fig = plt.figure()
ax = Axes3D(fig)

angle = 5*pi/180
x = [0,216,216,0]

y = [0,0,294*sin(angle),sin(angle)*294]
z = [0,0,cos(angle)*294,cos(angle)*294]
verts = [zip(x, y,z)]

#fig.set_label(s)
plt.ylabel('y')
plt.xlabel('x')

#print verts
ax.add_collection3d(Poly3DCollection(verts))



plt.axis([0,300,0,300])


plt.draw()
plt.show(block=False)

#i = numpy.divide(verts,10)
#j = i
i=0
"""
i=0
raw_input()


holder = [[50.0,0.0,0.0],
[60.7589027394,14.7,1.28118941839],
[71.5178054788,29.4,2.56237883678],
[82.2767082182,44.1,3.84356825517],
[93.0356109576,58.8,5.12475767356],
[103.794513697,73.5,6.40594709195],
[114.553416436,88.2,7.68713651034],
[125.312319176,102.9,8.96832592873],
[136.071221915,117.6,10.2495153471],
[146.830124655,132.3,11.5307047655],
[157.589027394,147.0,12.8118941839],
[168.347930133,161.7,14.0930836023],
[179.106832873,176.4,15.3742730207],
[189.865735612,191.1,16.6554624391],
[200.624638351,205.8,17.9366518575],
[211.383541091,220.5,19.2178412759],
[222.14244383,235.2,20.4990306942],
[232.90134657,249.9,21.7802201126],
[243.660249309,264.6,23.061409531],
[254.419152048,279.3,24.3425989494]]




i=0
for w in range(1):
	time.sleep(1)
	holder2 = holder[w]
    
	ax.scatter(holder2[0],holder2[1],holder2[2], color = (0.3600,0.6000,0.5000,1))
	plt.draw()
	
	#x2 = np.arange(9.0)
	#print x2
	i+=1
	#print "test"
	#print i
	if not(i%100):
		raw_input()
	#x1 = np.arange(9.0).reshape((3, 3))
	#print x1
	#i = numpy.add(i,j)
	

plt.show()
print "hello"

#r = [50,]
