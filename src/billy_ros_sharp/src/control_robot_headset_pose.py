#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist


class ControlRobotFromHeadsetSpeed:
	def __init__(self):
		rospy.init_node('control_robot')
		#self.count = 0
		#self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.headsetSpeed_sub = rospy.Subscriber('/headsetSpeed', Twist, self.headsetSpeed)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.robotSpeed = Twist()
		

	def headsetSpeed(self, msg):
		self.robotSpeed.linear.x = msg.angular.x / 10 
		self.robotSpeed.angular.z = msg.angular.y / 10
		print('new Twist get !')
		self.cmd_vel.publish(self.robotSpeed)

if __name__ == '__main__':
	ControlRobotFromHeadsetSpeed()
	rospy.spin()