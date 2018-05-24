#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

angle_range = 180
car_length = 1.5
desired_trajectory = 0.8 # 0.7
vel = 15

targetDist = 1.2

#errorList = [0,0,0,0,0]
error = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=10)
forwardPub = rospy.Publisher('forward', pid_input, queue_size=10)

def getRange(data,angle):
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]

def movingAverage(error):
	global errorList
	errorList.append(error)
	del errorList[0]
	return sum(errorList)/len(errorList)

def callback(data):
	# Wall following
	global error
	a = getRange(data,50)
	b = getRange(data,0)
	swing = math.radians(50)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	curr_dist = b*math.cos(alpha)
	future_dist = curr_dist+car_length*math.sin(alpha)
	error = desired_trajectory - future_dist
	
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)

	# Forward PID
	x = getRange(data,87)#Maybe look to left (84-90)
	y = getRange(data,90)
	z = getRange(data,93)
	minDist = min(x,y,z) #Find the closest distance in your cone
	error_f = minDist - targetDist
	fMsg = pid_input()
	fMsg.pid_error = error_f
	fMsg.pid_vel = 0
	forwardPub.publish(fMsg)

	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
