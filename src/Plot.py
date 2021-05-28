#!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse 
import numpy as np
from numpy import linalg as LA
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

PI = 3.1415926
s = 5.9915*4

zoom = 1
plt_get = False
temp = 0

pose = np.array([10.5*0.395+12.0,7.5*0.395+12.0,PI/2])
theta = 0
'''
LM_Blue_X = [4,6,8,9,9,8,6,4,3,3]
LM_Blue_Y = [4,3,4,6,9,11,12,11,9,6]
LM_Yellow_X = [3,5,7,9,11,12,12,11,9,7,5,3,1,0,0,1,0,12,12,0]
LM_Yellow_Y = [1,0,0,1,3,6,9,12,14,15,15,14,12,9,6,3,0,0,15,15]
'''
LM_Yellow_X = [1,1,1,1,2,2,2,3,3,5,5,7,7,9,9,10,10,10,11,11,11,11,0,12,12,0]
LM_Yellow_Y = [4,6,8,10,2,7,12,1,13,0,14,0,14,1,13,2,7,12,4,6,8,10,0,0,15,15 ]
LM_Blue_X = [ 3,3,4,4,4,4,6,6,8,8,8,8,9,9 ]
LM_Blue_Y = [ 5,9,3,6,8,11,2,12,3,6,8,11,5,9 ]
##
#goal = [[16.1,16],[15.4,16.9],[14.3,17.5],[13.2,16.9],[12.7,16],[12.6,15],[12.7,14],[13.2,13.1],[14.3,12.5],[15.4,13.1],[16.1,14],[16.2,15],[16.1,16]]
goal = [[10,9],[8.7,11.7],[6,13],[3.3,11.7],[2,9],[3,7],[2,5],[3.3,2.3],[6,1],[8.7,2.3],[10,5],[9,7]]
for i in range(len(LM_Blue_X)):
	LM_Blue_X[i] = LM_Blue_X[i]*0.395+12
for i in range(len(LM_Blue_Y)):
	LM_Blue_Y[i] = LM_Blue_Y[i]*0.395+12
for i in range(len(LM_Yellow_X)):
	LM_Yellow_X[i] = LM_Yellow_X[i]*0.395+12
for i in range(len(LM_Yellow_Y)):
	LM_Yellow_Y[i] = LM_Yellow_Y[i]*0.395+12
for i in range(len(goal)):
	goal[i][0] = goal[i][0]*0.395+12
	goal[i][1] = goal[i][1]*0.395+12
#global cone_no
cone_no = 0#camera observed cone number
coneX = np.zeros(30)#camera observed cone distance
coneP = np.zeros(30)#camera observed cone angle 
coneC = np.zeros(30)#camera observed cone color

def cone_callback(msg):
	global cone_no
	cone_no = len(msg.data)/3
	
	for i in range(cone_no):
		coneX[i] = msg.data[3*i]
		coneP[i] = msg.data[3*i+1]
		coneC[i] = msg.data[3*i+2]

def callback(msg):
	pose[0] = msg.data[0]
	pose[1] = msg.data[1]	
	pose[2] = msg.data[2]

#	global plt_
#	plt_ = np.zeros(len(msg.data)-3)
#	for i in range(len(msg.data)-3):
#		plt_[i] = msg.data[3+i]
#	global plt_get
#	plt_get = True


if __name__ == '__main__':
	rospy.init_node('Plot', anonymous=True)
	rospy.Subscriber('status',Float64MultiArray,callback)
	rospy.Subscriber('coneXP',Float64MultiArray,cone_callback)

	rate = rospy.Rate(20) # 100hz
	while not rospy.is_shutdown():
		Cone_blue_X = []
		Cone_blue_Y = []
		Cone_yellow_X = []
		Cone_yellow_Y = []

		print(cone_no)
		if cone_no > 0:
			for i in range(cone_no):
				ang = coneP[i]+pose[2]
				if ang > PI:
					ang -= 2*PI
				if ang < -PI:
					ang += 2*PI
				if coneC[i] == 0:#blue
					Cone_blue_X.append(pose[0]+math.cos(ang)*coneX[i])
					#Cone_blue_X += [[pose[0]+math.cos(ang)*coneX[i]]]
					Cone_blue_Y.append(pose[1]+math.sin(ang)*coneX[i])
				if coneC[i] == 1:#yellow
					Cone_yellow_X.append(pose[0]+math.cos(ang)*coneX[i])
					Cone_yellow_Y.append(pose[1]+math.sin(ang)*coneX[i])

		
		plt.clf()

		plt.plot(LM_Blue_X,LM_Blue_Y,'b^',LM_Yellow_X,LM_Yellow_Y,'y^',pose[0],pose[1],'ro',[pose[0],pose[0]+0.5*math.cos(pose[2])],[pose[1],pose[1]+0.5*math.sin(pose[2])],'r',Cone_blue_X,Cone_blue_Y,'ro',Cone_yellow_X,Cone_yellow_Y,'ro')
		#plt.plot()
		#for ii in range(len(goal)):
		#	print("zdkjfrhndmslnlkdn")
		#	plt.scatter(goal[ii][0],goal[ii][1])
		plt.axis('equal')
		plt.xlim([11,18])
		plt.ylim([11,19])		
	
		
		plt.draw()
		plt.pause(0.0001)
		
		rospy.loginfo(rospy.get_time()-temp)
		temp = rospy.get_time()-temp
		
		rate.sleep()
	
