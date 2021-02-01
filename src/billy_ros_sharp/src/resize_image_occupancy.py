#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Vector3
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
class GenerateVisualData():
	def __init__(self):
		rospy.init_node("map_creator", anonymous=True)
		self.bridge = CvBridge()
		#for turtlebot3 waffle
		#image_topic = '/camera/rgb/image_raw/compressed'
		#for occupancy topic
		self.image_topic = '/map'
		self.resolution = 0.05
		self.rectangle_odom = 5
		self.listMeanMarker = []
		#image_topic = '/camera/rgb/image_raw/'
		self.robot_odom_value = Odometry()
		self.image_sub = rospy.Subscriber('/image_from_occupancy', Image, self.image_map_callback)
		self.vive_left_button_sub = rospy.Subscriber('/left_vive_button', Vector3, self.vive_left_button_callback)
		self.imge_robot_sub = rospy.Subscriber('/camera/rgb/image_raw/', Image, self.image_robot_callback)
		self.visual_image_pub = rospy.Publisher('/visual_augmented_image', Image, queue_size=10)
		self.image_message = Image()
		self.image = np.zeros((384, 384, 1), np.uint8)
		self.image_map = Image()
		self.vive_left_button = Vector3()
		self.scale = 0.4
	

	def image_map_callback(self, image_occupancy):
		# We want to resize the image to add it in the visual field of view
		x_offset = 0
		y_offset = 0

		self.image_map = image_occupancy
		"""
		cv_image = self.bridge.imgmsg_to_cv2(self.image_map, "bgr8")
		height = int(cv_image.shape[1] * self.scale)
		width = int(cv_image.shape[0] * self.scale)
		dsize = (width, height)
		#print(dsize) # (76, 76)
		small_image = cv2.resize(cv_image, dsize)

		cv_image[y_offset:y_offset+width, x_offset:x_offset+height] = small_image
		image_to_send = cv_image
		#print(image_to_send.shape)
		image_message = self.bridge.cv2_to_imgmsg(image_to_send, "bgr8")
		self.visual_imgage_pub.publish(image_message)
		"""

	def image_robot_callback(self, image_robot):

		self.image_robot = image_robot
		x_offset = 0
		y_offset = 0
		cv_image_map = self.bridge.imgmsg_to_cv2(self.image_map, "bgr8")
		cv_image_robot = self.bridge.imgmsg_to_cv2(image_robot, "bgr8")
		
		height = int(cv_image_map.shape[1] * self.scale)
		width = int(cv_image_map.shape[0] * self.scale)
		dsize = (width, height)
		#print(dsize) # (76, 76)
		small_image = cv2.resize(cv_image_map, dsize)

		cv_image_robot[y_offset:y_offset+width, x_offset:x_offset+height] = small_image
		image_to_send = cv_image_robot
		#print(image_to_send.shape)
		image_message = self.bridge.cv2_to_imgmsg(image_to_send, "bgr8")
		if(self.vive_left_button.x == 1):
			self.visual_image_pub.publish(image_message)
			print("Show the map")
		else:
			self.visual_image_pub.publish(image_robot)
		
	
	def vive_left_button_callback(self, ros_msg):
		self.vive_left_button = ros_msg

def main(args):
	GenerateVisualData()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
