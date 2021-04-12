#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class ControlRobotJoy:

	def __init__(self):
		rospy.init_node('control_robot_joy')
		print("control of tunrning mount and link 1 ")
		self.rate = rospy.Rate(10)
		self.start_time = rospy.get_rostime()
		
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
		self.joy_c = Joy
		self.pub_joint1 = rospy.Publisher("/billy_arm/joint1_position_controller/command",Float64, queue_size=1)
		self.pub_joint2 = rospy.Publisher("/billy_arm/joint2_position_controller/command",Float64, queue_size=1)
		self.pub_joint3 = rospy.Publisher("/billy_arm/joint3_position_controller/command",Float64, queue_size=1)
		self.pub_joint4 = rospy.Publisher("/billy_arm/joint4_position_controller/command",Float64, queue_size=1)
		self.pub_joint5 = rospy.Publisher("/billy_arm/joint5_position_controller/command",Float64, queue_size=1)
		self.pub_joint6 = rospy.Publisher("/billy_arm/joint6_position_controller/command",Float64, queue_size=1)
		self.joint1=Float64()
		self.joint2=Float64()
		self.joint3=Float64()
		self.joint4=Float64()
		self.joint5=Float64()
		self.joint6=Float64()
		self.theta1=0
		self.theta2=0
		self.theta3=0
		self.theta4=0
		self.theta5=0
		self.theta6=0
		self.gripper_open= False
		self.joint_selector=1
		self.joint_selector_2=1


	def joy_callback(self, msg):
		
		lr=msg.axes[6]
		tb=msg.axes[7]
		gripper=msg.buttons[0]
		#self.joint_selection()

		decre_1=msg.buttons[4]
		incre_1=msg.buttons[5]

		incre_2=msg.axes[5]
		decre_2=msg.axes[2]



		if(decre_1==1):
			self.joint_selector=self.joint_selector-1
			if(self.joint_selector<1):
				self.joint_selector=1
			self.print_joint_selection()
			

		elif(incre_1==1):
			self.joint_selector=self.joint_selector+1	
			if(self.joint_selector>3):
				self.joint_selector=3
			self.print_joint_selection()
			#print("control of joint " +str(self.joint_selector))

		if(decre_2==-1):
			self.joint_selector_2=1
			print("control of tunrning mount ")
			

		elif(incre_2==-1):
			self.joint_selector_2=4
			print("control of rotational part end effector")
		
			

		if (lr==1):

			if(self.joint_selector_2==1):
				self.theta1=self.theta1+0.1
				print(self.theta1)
				self.pub_joint1.publish(self.theta1)
				self.joint1.data = self.theta1
			elif(self.joint_selector_2==4):
				self.theta4=self.theta4+0.1
				print(self.theta4)
				self.pub_joint4.publish(self.theta4)
				self.joint4.data = self.theta4

		elif(lr==-1):
			if(self.joint_selector_2==1):
				self.theta1=self.theta1-0.1
				print(self.theta1)
				self.pub_joint1.publish(self.theta1)
				self.joint1.data = self.theta1
			elif(self.joint_selector_2==4):
				self.theta4=self.theta4-0.1
				print(self.theta4)
				self.pub_joint4.publish(self.theta4)
				self.joint4.data = self.theta4
			

		if (tb==1):
				if(self.joint_selector==1):
					self.theta2=self.theta2+0.1
					print(self.theta2)
					self.pub_joint2.publish(self.theta2)
					self.joint2.data = self.theta2
				elif(self.joint_selector==2):
					self.theta3=self.theta3+0.1
					print(self.theta3)
					self.pub_joint3.publish(self.theta3)
					self.joint3.data = self.theta3
				elif(self.joint_selector==3):
					self.theta5=self.theta5+0.1
					print(self.theta5)
					self.pub_joint5.publish(self.theta5)
					self.joint5.data = self.theta5

		elif(tb==-1):
			if(self.joint_selector==1):
					self.theta2=self.theta2-0.1
					print(self.theta2)
					self.pub_joint2.publish(self.theta2)
					self.joint2.data = self.theta2
			elif(self.joint_selector==2):
					self.theta3=self.theta3-0.1
					print(self.theta3)
					self.pub_joint3.publish(self.theta3)
					self.joint3.data = self.theta3
			elif(self.joint_selector==3):
					self.theta5=self.theta5-0.1
					print(self.theta5)
					self.pub_joint5.publish(self.theta5)
					self.joint5.data = self.theta5

		if(gripper==1):

			if(self.gripper_open==False):

				print("Open Gripper")
				self.theta7=-0.8
				self.pub_joint6.publish(self.theta7)
				self.gripper_open=True
				self.joint6.data = self.theta7

			elif(self.gripper_open==True):

				self.theta7=0
				print("Gripper close")
				self.pub_joint6.publish(self.theta7)
				self.joint6.data = self.theta7
				self.gripper_open= False

	def joint_selection (self):

		count_1=self.joy_c.buttons[4]
		count_2=self.joy_c.buttons[5]
		if(count_1==1):
			self.joint_selector=self.joint_selector-1
			if(joint_selector<1):
				self.joint_selector=1
		elif(count_2==1):
			self.joint_selector=self.joint_selector+1

		elif(joint_selector>3):
			self.joint_selector=3
		print("control of joint " +str(self.joint_selector))

	def print_joint_selection(self):
		if(self.joint_selector==1):
					print("contol of link 1")
					
		elif(self.joint_selector==2):
					print("contol of link 2")
					
		elif(self.joint_selector==3):
					print("contol of link 4")


			

if __name__ == '__main__':
	ControlRobotJoy()
	rospy.spin()
	#rosparam set joy_node/dev "/dev/input/js2"
