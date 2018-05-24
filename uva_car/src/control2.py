#!/usr/bin/env python

"""
Control2.py - PID controller for wall following, velocity control, and distance to a lead car.
    Node: pid_controller
    Subscribes to: /error, /velocityY
    Publishes to: /drive_parameters
"""
import rospy

from uva_car.msg import drive_param
from uva_car.msg import pid_input
from uva_car.msg import drive_velocity
import math

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# PID Values
# Wall following
kp = 18.0 #12
kd = 0.12 #0.01
ki = 2.56 # 0.0

# Target Velocity
kp_vel = 40.0
kd_vel = 0.09 #0.01
ki_vel =0 # 2

# Forward control
kp_f = 15.0
kd_f = 0.0
ki_f = 0.0

# Global variables
servo_offset = 0 #18.5 # zero correction offset in case servo is misaligned.
prev_error = 0.0
prev_vE = 0.0
error = 0.0
integral = 0.0
vIntegral = 0.0

targetV = 0.4
carV = 0
maxVel = 20
minVel = 14

prev_error_f = 0.0

# PID Control for wall following
def wallControl(data):
    global integral
    global prev_error

    angle = servo_offset
    error = 5.0*data.pid_error
    if error!=0.0:
        if abs(error - prev_error)>0.5:
            integral = integral + error
        control_error = kp*error + kd*(prev_error - error) + ki*integral
        integral = integral/1.3

        #print "Control error",control_error
        angle = angle - control_error

    elif error == 0.0:
        angle = servo_offset


    prev_error = error

    # Control bounds of angle
    if angle<-100:
        angle = -100
    if angle>100:
        angle = 100

    # Create drive parameters message
    msg = drive_param()
    msg.angle = angle
    msg.velocity = carV
    #msg.velocity = 18
    pub.publish(msg)

# PID Control for Target Velocity
def velocityControl(data):
    global prev_vE
    global carV
    global vIntegral

    velocity = data.yVelocity

    if velocity < 0:
        velocity = 0

    verror = targetV - velocity
    if abs(verror - prev_vE)>0.5:
        vIntegral = vIntegral + verror
    control_error_vel = kp_vel*verror + kd_vel*(prev_vE - verror) + ki_vel*vIntegral

    prev_vE = verror

    #carV =  carV - control_error_vel
    #carV = 18

    # Limit Range of car velocity
    if control_error_vel > maxVel:
        carV = maxVel
    elif control_error_vel < minVel:
        carV = minVel
    else:
        carV = control_error_vel

# PID Control for Forward Following
def forwardControl(data):
    global targetV
    global prev_error_f

    error = data.pid_error
    if error!=0.0:
        control_error = kp_f*error + kd_f*(prev_error_f - error) #+ ki_f*integral

    prev_error_f = error

    if control_error < 0:
        targetV = 0
    elif control_error > 4:
        targetV = 4
    else:
        targetV = control_error

def listener():
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", pid_input, wallControl)
    rospy.Subscriber("velocityY", drive_velocity, velocityControl)
    # rospy.Subscriber("forward", pid_input, forwardControl) # Uncomment for follower vehicles
    rospy.spin()


if __name__ == '__main__':
    print("Listening to error for PID")
    listener()
