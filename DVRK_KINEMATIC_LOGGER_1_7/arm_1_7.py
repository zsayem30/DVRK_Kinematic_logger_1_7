#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import cv2
import numpy as np
import dvrk 
import sys

class robot:

	def __init__(self, robot_name, ros_namespace = '/dvrk/'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.__robot_name = robot_name
		self.__ros_namespace = ros_namespace
		self.__position_joint_current = []
		self.__measured_joint = []
		self.__position_jaw_current = []
		self.__position_cartesian_current = 0
		self.__orientation_current = []
		self.seq = 0
		self.rate = rospy.Rate(2)
		self.count = -1
		self.nsecs = 0
		self.secs = 0
		self.start_time = 0
		self.full_array = []

		full_ros_namespace = self.__ros_namespace + self.__robot_name

		#subscriber
		rospy.Subscriber(full_ros_namespace + '/state_joint_current', JointState, self.current_joint_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/state_jaw_current', JointState, self.current_jaw_state_callback, queue_size = 1, buff_size = 1)
		#callback

	def current_joint_state_callback(self, data):
		self.__position_joint_current[:] = data.position
		self.full_array.append(self.__position_joint_current[:])
		self.seq = data.header.seq
		self.nsecs = data.header.stamp.nsecs
		self.secs = data.header.stamp.secs
		self.start_time = 0

	def current_jaw_state_callback(self, data):
		self.__position_jaw_current[:] = data.position

	#getters
	def get_current_joint_position(self):
		
		return self.__position_joint_current

	def get_current_jaw_position(self):
		
		return self.__position_jaw_current
		
	def get_seq(self):
		
		return self.seq

	def increment_count(self):
		
		self.count = self.count + 1
		
		return self.count

	def get_nsecs(self):
		
		return self.nsecs*10**(-9)

	def get_secs(self):
		
		return self.secs

	def get_time(self):

		self.time = self.get_secs() + self.get_nsecs()
		return self.time

	def get_start_time(self):
		
		if self.count == 1:
			self.start_time = self.get_time()

		return self.start_time

	def get_count(self):
		return self.count

	def get_full_array(self):
		return self.full_array