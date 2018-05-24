#!/usr/bin/env python

import rospy

from race.msg import drive_param
from race.msg import pid_input
import math

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

kp = 12.0
kd = 0.01
kp_vel = 42.0
kd_vel = 0.0

ki = 0.0
servo_offset = 18.5 # zero correction offset in case servo is misaligned.
prev_error = 0.0 
error = 0.0
integral = 0.0
vel_input = 25.0


def control(data):
	global integral
	global prev_error
	print integral
	global vel_input
	global kp
	global ki
	global kd
	global kd_vel
	global kp_vel
	velocity = vel_input
	angle = servo_offset
	error = 5.0*data.pid_error
	time = data.time
	print(time)
	yVel = data.drive_velocity
	print(yVel)
	print "Error Control",error
	if error!=0.0:
		# if abs(error - prev_error)>0.5: 	
		# 	integral = integral + error	
		control_error = kp*error + kd*(prev_error - error)# + ki*integral
		integral = integral/1.3
		
		print "Control error",control_error
		angle = angle - control_error

		control_error_vel = kp_vel*error + kd_vel*(prev_error - error)
		print "Control error velocity",control_error_vel

		velocity = velocity - abs(control_error_vel)/10



	elif error == 0.0:
		angle = servo_offset
		

	prev_error = error

	if angle<-100:
		angle = -100
	if angle>100:
		angle = 100
	print "Velocity",velocity
	print "Angle",angle
	msg = drive_param();
	if velocity < 14:
		velocity = 14
	if velocity > 17:
		velocity = 17
	msg.velocity = velocity
	
	msg.angle = angle
	pub.publish(msg)


def listener():
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.Subscriber("velocityY", drive_velocity, control)
	rospy.Subscriber("velocityY", time, control)
	global kp
	global ki
	global kd
	global vel_input
	#kp = input("Enter Kp Value: ")
	#ki = input("Enter Ki Value: ")
	#kd = input("Enter Kd Value: ")
	#vel_input = input("Enter Velocity: ")
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
