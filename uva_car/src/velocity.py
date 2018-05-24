#!/usr/bin/env python

import rospy
import math
#import time 
from sensor_msgs.msg import Imu
from uva_car.msg import drive_velocity
desired_trajectory = 1
vel = 30

pub = rospy.Publisher('velocityY', drive_velocity, queue_size=10)
#acc = 0
count = 0
sumAcc = 0.0
v0 = 0
t0 = 0
biasCount = 0
bias = 0

accList = [0,0,0,0,0]

def movingAverage(acc):
	global accList
	accList.append(acc)
	del accList[0]
	return sum(accList)/len(accList)

def callback(data):
	#yAcc = data.linear_acceleration.y
	global count
	global biasCount
	global bias
	global sumAcc
	global v0
	global t0

	if biasCount <= 1000:
		biasCount = biasCount + 1
		bias = bias + data.linear_acceleration.y
		if biasCount == 1000:
			bias = bias / 1001
			t0 = rospy.get_rostime()
	else:
		#count = count+1;
		#sumAcc = sumAcc + data.linear_acceleration.y - bias
		#if count == 2:
			#avgAcc = sumAcc / 3.0
			#sumAcc = 0
		
			#velocity = v0 + 0.06 * avgAcc
			#if velocity < 0:
			#	velocity = 0
			#v0 = velocity
		
		# KALMAN?
		acc = movingAverage(data.linear_acceleration.y) - bias
		#t1 = rospy.get_rostime().nsecs/1e9
		t1 = rospy.get_rostime()
		duration = (t1-t0).to_sec()
		velocity = v0 +	(duration)*(acc)
		v0 = velocity
		t0 = t1

		msg = drive_velocity()		
		print("Velocity: ", velocity)
		print("Time In Nano: ", rospy.get_rostime().nsecs)
		msg.yVelocity = velocity
		msg.time = 0
		pub.publish(msg)
			#count = 0
		
	

if __name__ == '__main__':
	print("Velocity node started")
	rospy.init_node('velocity',anonymous = False)
	rospy.Subscriber("imu",Imu,callback)
	#rospy.Timer(rospy.Duration(1), callback)
	#rospy.Subscriber("imu", Imu, calback)
	rospy.spin()
