#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

class Kalman(object):

	def __init__(self, n_states, n_sensors):
		super(Kalman, self).__init__()
		self.n_states = n_states
		self.n_sensors = n_sensors
		self.x = np.matrix(np.zeros(shape=(n_states,1)))
		self.P = np.matrix(np.identity(n_states))
		self.F = np.matrix(np.identity(n_states))
		self.u = np.matrix(np.zeros(shape=(n_states,1)))
		self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))
		self.R = np.matrix(np.identity(n_sensors))
		self.I = np.matrix(np.identity(n_states))
		self.first = True

	def update(self, Z):
		w = Z - self.H * self.x
		S = self.H * self.P * self.H.getT() + self.R
		K = self.P * self.H.getT() * S.getI()
		self.x = self.x + K * w
		self.P = (self.I - K * self.H) * self.P

	def predict(self):
		self.x = self.F * self.x + self.u
		self.P = self.F * self.P * self.F.getT()

class Subscriber(object):

	def __init__(self):
		super(Subscriber, self).__init__()
		rospy.init_node('imu_conv', anonymous=True)
		self.kalman = Kalman(n_states = 3, n_sensors = 3)
		self.kalman.H = np.matrix(np.identity(self.kalman.n_states))
		# self.kalman.P *= 10
		self.kalman.R *= 0.04 #covariance
		self.pub_accel = rospy.Publisher('kalman', Vector3, queue_size=10)
		#rospy.Subscriber('accelerometer', Vector3, self.callback_accel)
		rospy.Subscriber("imu",Imu, self.callback_accel)
		rospy.spin()

	def callback_accel(self, data):
		# print "received data: ", data
		#Z = np.matrix([data.x,data.y,data.z]).getT()
		Z = np.matrix([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z]).getT()
		if self.kalman.first:
			self.kalman.x = Z
			self.kalman.first = False

		self.kalman.update(Z)
		self.kalman.predict()
		vec = Vector3()
		vec.x = self.kalman.x[0]
		vec.y = self.kalman.x[1]
		vec.z = self.kalman.x[2]
		self.pub_accel.publish(vec)

if __name__ == '__main__':
	subscriber = Subscriber()

