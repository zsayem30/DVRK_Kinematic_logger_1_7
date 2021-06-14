#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import cv2
import numpy as np
import xlsxwriter
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R
import os
import arm
import camera
import dvrk

if sys.version_info.major < 3:
    input = raw_input

if __name__ == '__main__':

	rospy.init_node('topic_publisher')
	rate = rospy.Rate(140)

	PSM1 = arm.robot('PSM1')
	PSM2 = arm.robot('PSM2')
	ECM = arm.robot('ECM')
	
	psm1 = dvrk.psm("PSM1")
	psm2 = dvrk.psm("PSM2")
	ecm = dvrk.ecm("ECM")

	left_cam = camera.camera('left')
	right_cam = camera.camera('right')

	input("		Press Enter to start logging...")

	workbook = xlsxwriter.Workbook('dvrk_kinematic_logger_test.xlsx')
	worksheet = workbook.add_worksheet()

	#start from the first cell
	#write the column headers for PSM1
	worksheet.write(0, 0, 'Time (Seconds)')
	worksheet.write(0, 1, 'Frame Number')
	worksheet.write(0, 2, 'PSM1_joint_1')
	worksheet.write(0, 3, 'PSM1_joint_2')
	worksheet.write(0, 4, 'PSM1_joint_3')
	worksheet.write(0, 5, 'PSM1_joint_4')
	worksheet.write(0, 6, 'PSM1_joint_5')
	worksheet.write(0, 7, 'PSM1_joint_6')
	worksheet.write(0, 8, 'PSM1_jaw_angle')
	worksheet.write(0, 9, 'PSM1_ee_x')
	worksheet.write(0, 10, 'PSM1_ee_y')
	worksheet.write(0, 11, 'PSM1_ee_z')

	worksheet.write(0, 12, 'PSM1_Orientation_Matrix_[1,1]')
	worksheet.write(0, 13, 'PSM1_Orientation_Matrix_[1,2]')
	worksheet.write(0, 14, 'PSM1_Orientation_Matrix_[1,3]')

	worksheet.write(0, 15, 'PSM1_Orientation_Matrix_[2,1]')
	worksheet.write(0, 16, 'PSM1_Orientation_Matrix_[2,2]')
	worksheet.write(0, 17, 'PSM1_Orientation_Matrix_[2,3]')

	worksheet.write(0, 18, 'PSM1_Orientation_Matrix_[3,1]')
	worksheet.write(0, 19, 'PSM1_Orientation_Matrix_[3,2]')
	worksheet.write(0, 20, 'PSM1_Orientation_Matrix_[3,3]')


	#write the column headers for PSM2
	worksheet.write(0, 21, 'PSM2_joint_1')
	worksheet.write(0, 22, 'PSM2_joint_2')
	worksheet.write(0, 23, 'PSM2_joint_3')
	worksheet.write(0, 24, 'PSM2_joint_4')
	worksheet.write(0, 25, 'PSM2_joint_5')
	worksheet.write(0, 26, 'PSM2_joint_6')
	worksheet.write(0, 27, 'PSM2_jaw_angle')

	worksheet.write(0, 28, 'PSM2_ee_x')
	worksheet.write(0, 29, 'PSM2_ee_y')
	worksheet.write(0, 30, 'PSM2_ee_z')

	worksheet.write(0, 31, 'PSM2_Orientation_Matrix_[1,1]')
	worksheet.write(0, 32, 'PSM2_Orientation_Matrix_[1,2]')
	worksheet.write(0, 33, 'PSM2_Orientation_Matrix_[1,3]')

	worksheet.write(0, 34, 'PSM2_Orientation_Matrix_[2,1]')
	worksheet.write(0, 35, 'PSM2_Orientation_Matrix_[2,2]')
	worksheet.write(0, 36, 'PSM2_Orientation_Matrix_[2,3]')

	worksheet.write(0, 37, 'PSM2_Orientation_Matrix_[3,1]')
	worksheet.write(0, 38, 'PSM2_Orientation_Matrix_[3,2]')
	worksheet.write(0, 39, 'PSM2_Orientation_Matrix_[3,3]')

	#write the column headers for ECM (4 joints)
	worksheet.write(0, 40, 'ECM_joint_1')
	worksheet.write(0, 41, 'ECM_joint_2')
	worksheet.write(0, 42, 'ECM_joint_3')
	worksheet.write(0, 43, 'ECM_joint_4')

	worksheet.write(0, 44, 'ECM_ee_x')
	worksheet.write(0, 45, 'ECM_ee_y')
	worksheet.write(0, 46, 'ECM_ee_z')

	worksheet.write(0, 47, 'ECM_Orientation_Matrix_[1,1]')
	worksheet.write(0, 48, 'ECM_Orientation_Matrix_[1,2]')
	worksheet.write(0, 49, 'ECM_Orientation_Matrix_[1,3]')

	worksheet.write(0, 50, 'ECM_Orientation_Matrix_[2,1]')
	worksheet.write(0, 51, 'ECM_Orientation_Matrix_[2,2]')
	worksheet.write(0, 52, 'ECM_Orientation_Matrix_[2,3]')

	worksheet.write(0, 53, 'ECM_Orientation_Matrix_[3,1]')
	worksheet.write(0, 54, 'ECM_Orientation_Matrix_[3,2]')
	worksheet.write(0, 55, 'ECM_Orientation_Matrix_[3,3]')
	#write the column headers for the cameras
	worksheet.write(0, 56, 'Left Camera Image')
	worksheet.write(0, 57, 'Right Camera Image')

	i = 0

	start_time = 0

	def callback_dummy(data):
		global i, start_time

		PSM1_jp = PSM1.get_current_joint_position()
		PSM1_jaw_angle = PSM1.get_current_jaw_position()
		PSM1_ee = [psm1.get_current_position().p.x(), psm1.get_current_position().p.y(), psm1.get_current_position().p.z()]

		PSM1_matrix = [psm1.get_current_position().M[0,0], psm1.get_current_position().M[0,1], psm1.get_current_position().M[0,2], psm1.get_current_position().M[1,0], psm1.get_current_position().M[1,1], psm1.get_current_position().M[1,2], psm1.get_current_position().M[2,0], psm1.get_current_position().M[2,1], psm1.get_current_position().M[2,2]]

		PSM2_jp = PSM2.get_current_joint_position()
		PSM2_jaw_angle = PSM2.get_current_jaw_position()
		PSM2_ee = [psm2.get_current_position().p.x(), psm2.get_current_position().p.y(), psm2.get_current_position().p.z()]
		PSM2_matrix = [psm2.get_current_position().M[0,0], psm2.get_current_position().M[0,1], psm2.get_current_position().M[0,2], psm2.get_current_position().M[1,0], psm2.get_current_position().M[1,1], psm2.get_current_position().M[1,2], psm2.get_current_position().M[2,0], psm2.get_current_position().M[2,1], psm2.get_current_position().M[2,2]]

		ECM_jp = ECM.get_current_joint_position()
		ECM_ee = [ecm.get_current_position().p.x(), ecm.get_current_position().p.y(), ecm.get_current_position().p.z()]
		ECM_Orientation_Matrix = [ecm.get_current_position().M[0,0], ecm.get_current_position().M[0,1], ecm.get_current_position().M[0,2], ecm.get_current_position().M[1,0], ecm.get_current_position().M[1,1], ecm.get_current_position().M[1,2], ecm.get_current_position().M[2,0], ecm.get_current_position().M[2,1], ecm.get_current_position().M[2,2]]
		image_left = left_cam.get_image()
		image_right = right_cam.get_image()

		PSM1_data = np.concatenate((PSM1_jp, PSM1_jaw_angle, PSM1_ee, PSM1_Orientation_Matrix), axis = 0)
		PSM2_data = np.concatenate((PSM2_jp, PSM2_jaw_angle, PSM1_ee, PSM1_Orientation_Matrix), axis = 0)
		ECM_data = np.concatenate((ECM_jp, ECM_ee, ECM_Orientation_Matrix), axis = 0)
		all_data = np.concatenate((PSM1_data, PSM2_data, ECM_data), axis = 0)

		time = data.header.stamp.secs + data.header.stamp.nsecs*10**(-9)

		if i != 0:
			Sequence = i
			time = time - start_time
			info_frame = [time, Sequence]

			all_data = np.concatenate((info_frame, PSM1_data, PSM2_data, ECM_data), axis = 0)

			for col in range(len(all_data)):
				worksheet.write(i, col, all_data[col])

			image_saved_left = left_cam.save_image()

			if image_saved_left == True:
				worksheet.write(i, 56, "left"+"_Camera" +"_" + str(i)+".png")

			image_saved_right = right_cam.save_image()

			if image_saved_right == True:
				worksheet.write(i, 57, "right"+"_Camera" +"_" + str(i)+".png")

		else:
			start_time = time
		i = i + 1

	rospy.Subscriber('/dvrk/PSM1/state_jaw_current', JointState, callback_dummy, queue_size = 1, buff_size = 1000000)

	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print("Error Running ROS." + e)
		pass

	workbook.close()