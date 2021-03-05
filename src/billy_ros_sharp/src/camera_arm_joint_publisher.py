#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Vector3

class JointPub(object):
	def __init__(self):

		self.publishers_array = []
		self.joint_1_pub = rospy.Publisher('/billy_camarm/camarm_connector_bot_controller/command', Float64, queue_size=1)
		self.joint_2_pub = rospy.Publisher('/billy_camarm/camarm_ztox_position_controller/command', Float64, queue_size=1)
		
		
		self.publishers_array.append(self.joint_1_pub)
		self.publishers_array.append(self.joint_2_pub)
		
		self.pose_sub = rospy.Subscriber('/headset_Rotation', Vector3, self.headsetRotationCallback)
		self.init_pos = [0.0,0.0]
		self.actual_joint_pose = [0.0,0.0]
		self.headset_rotation = Vector3()
		
	def set_init_pose(self):
		"""
		Sets joints to initial position [0,0]
		:return:
		"""
		self.check_publishers_connection()
		self.move_joints(self.init_pos)


	def check_publishers_connection(self):
		"""
		Checks that all the publishers are working
		:return:
		"""
		rate = rospy.Rate(10)  # 10hz
		while (self.joint_1_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_1_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_1_pub Publisher Connected")

		while (self.joint_2_pub.get_num_connections() == 0):
			rospy.logdebug("No susbribers to joint_2_pub yet so we wait and try again")
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				# This is to avoid error when world is rested, time when backwards.
				pass
		rospy.logdebug("joint_2_pub Publisher Connected")
		rospy.logdebug("All Publishers READY")

	def joint_mono_des_callback(self, msg):
		rospy.logdebug(str(msg.joint_state.position))

		self.move_joints(msg.joint_state.position)

	def move_joints(self, joints_array):

		i = 0
		for publisher_object in self.publishers_array:
		  joint_value = Float64()
		  joint_value.data = joints_array[i]
		  rospy.logdebug("JointsPos>>"+str(joint_value))
		  publisher_object.publish(joint_value)
		  i += 1

	def headsetRotationCallback(self, ros_rotation):
		self.headset_rotation = ros_rotation
		#theta1 = - ros_rotation.x
		#theta1 = ros_rotation.z
		#theta2 = 3.14 - ros_rotation.y
		print("rotation y" + str(- ros_rotation.y))
		theta1 = -ros_rotation.y - 100
		theta2 = ros_rotation.z
		

		# Convert into radian
		theta1 = math.radians(theta1)
		theta2 = math.radians(theta2)

		print("angles have been calculated")
		self.actual_joint_pose = [theta1, theta2]
		print(str(self.actual_joint_pose))
		self.move_joints(self.actual_joint_pose)
	
if __name__=="__main__":
	rospy.init_node('camera_joint_publisher_node')
	joint_publisher = JointPub()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
