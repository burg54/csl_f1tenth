#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
import matplotlib.pyplot as plt

angle_range = 180
car_length = 1.5
desired_trajectory = 0.5 # old val 0.7
vel = 15
path = [1,0,1,1,1]
turn = 0
preva = 0
flag = 0
errorList = [0,0,0,0,0]
turnStarted = 0
error = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data,angle):
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]

def decideTurn(a,error):
	global nextTurn
	global path
	global preva
	global flag
	if path[nextTurn] == 1 or abs(error) < 0.5:
		rError = error
	else:
		rError = 0
	a = movingAverage(a)
	if a > 1.6 and preva<1.6:
                flag = 1
	if flag == 1 and a < -0.8:
		nextTurn = (nextTurn+1)%5
		flag = 0
        preva = a
	print "diff: ",a
	print "Path",nextTurn
	return rError

def decideTurn_slope(data):
	lower_index = range(10,20)
	upper_index = range(40,50,1)
	lower_range = []
	upper_range = []
	x_lower = []
	y_lower = []
	x_upper = []
	y_upper = []
	for i in range(0,len(lower_index)):
		lower_range.append(getRange(data,lower_index[i]))
		r = getRange(data,lower_index[i])
		theta = lower_index[i]
		x_lower.append(r*np.cos(math.radians(theta)))
		y_lower.append(r*np.sin(math.radians(theta)))
	for i in range(0,len(upper_index)):
		upper_range.append(getRange(data,upper_index[i]))
		r = getRange(data,upper_index[i])
		theta = upper_index[i]
		x_upper.append(r*np.cos(math.radians(theta)))
		y_upper.append(r*np.sin(math.radians(theta)))
	m1,a1 = np.polyfit(x_lower,y_lower,1)
	m2,a2 = np.polyfit(x_upper,y_upper,1)
	alpha1 = math.degrees(math.atan(m1))
	alpha2 = math.degrees(math.atan(m2))
	print "Slope_lower:", alpha1
	print "Sloper_upper:",alpha2
	print "difference :", (alpha2 - alpha1)

def decideTurn_new(error):
	global turn
	global turnStarted
	if turnStarted == 1:
		if path[turn] == 1:
			return error
			print("Execute turn")
		else:
			return 0
			print("Skip turn")
	return error

def detectTurn(alpha):
	global turnStarted
	global turn
	global error
	if turnStarted == 0 and math.degrees(alpha)>20:
		turnStarted = 1
		print "Turn Started"
	if turnStarted == 1 and math.degrees(abs(alpha)) < 10:
		turn = (turn+1)%len(path)
		turnStarted = 0
		print "Turn Ended"
	if turnStarted == 1:
		error = error

def movingAverage(error):
	global errorList
	errorList.append(error)
	del errorList[0]
	return sum(errorList)/len(errorList)

def callback(data):
	global error
	a = getRange(data,50)
	b = getRange(data,0)
	c = getRange(data,40)
	swing = math.radians(50)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	curr_dist = b*math.cos(alpha)
	future_dist = curr_dist+car_length*math.sin(alpha)
	error = desired_trajectory - future_dist
	detectTurn(alpha)
	error = decideTurn_new(error)
#	error = movingAverage(error)
#	error= decideTurn(c-b,error)
#	print("future distance: " + str(future_dist))
#	print("current distance: " + str(curr_dist))
#	print("error: " + str(error))
	print("alpha: " + str(math.degrees(alpha)))
#	print("Range at 0: " + str(b))
#	print("Range at 40: " + str(c))
#	print "Difference in ranges: ", c-b
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
