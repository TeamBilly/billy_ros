#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Vector3, Twist
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import sys
bridge = CvBridge()
"""
roslaunch My_own_code turtlebot3_vr_control.launch
roslaunch integrated_robotics_project start_navigation_turtlebot3_gmapping.launch
rosrun integrated_robotics_project generate_image_from_odom_occupancy.py
rosrun image_transport republish raw in:=/visual_augmented_image compressed out:=/visual_augmented_image
"""
class VrTeloperation():
	def __init__(self):
		rospy.init_node("dpad_teloperation", anonymous=True)
		self.bridge = CvBridge()
		
		self.vive_left_button_sub = rospy.Subscriber('/left_vive_dpad', Vector3, self.vive_left_dpad_callback)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.vive_left_dpad = Vector3()
		
		
	
	def vive_left_dpad_callback(self, ros_msg):
		if(ros_msg.x !=0 and ros_msg.y !=0):
			print('RobotMove')
		self.vive_left_dpad = ros_msg
		command = Twist()
		command.linear.x = ros_msg.x
		command.angular.z = ros_msg.y

		self.cmd_vel.publish(command)


def main(args):
	VrTeloperation()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
	

if __name__ == '__main__':
	main(sys.argv)
