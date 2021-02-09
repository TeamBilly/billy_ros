#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Joy

class ControlRobotJoy:
	def __init__(self):
		rospy.init_node('control_robot_joy')
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.robotSpeed = Twist()
		

	def joy_callback(self, msg):
		self.robotSpeed.linear.x = msg.axes[1]
		self.robotSpeed.angular.z = -msg.axes[0]
		print('new Twist get !')
		self.cmd_vel.publish(self.robotSpeed)

if __name__ == '__main__':
	ControlRobotJoy()
	rospy.spin()
