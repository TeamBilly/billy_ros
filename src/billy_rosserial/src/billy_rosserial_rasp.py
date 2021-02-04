#! /usr/bin/env python

import rospy
import tf
import math
#import numpy
#import cv2
#from math import *

from std_msgs.msg import Float32MultiArray, String, Int32, Int64, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist

from tf.transformations import quaternion_about_axis
#from tf.TransformBroadcaster import quaternion_about_axis

class Robot:
    
    def __init__ (self):
        #Node initialisation
        #rospy.init_node('final_assignment_node_adb')
        
        #Setting the frequency of robot to 10MHz
        self.rate = rospy.Rate(10)
        self.start_time = rospy.get_rostime()
        
        #Subscribers
        #self.sub_chatter = rospy.Subscriber('/chatter', String, self.callback_chatter)
        #self.sub_odomA = rospy.Subscriber('/odomA', Int64, self.callback_odomA)
        #self.sub_odomB = rospy.Subscriber('/odomB', Int64, self.callback_odomB)
        #self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd_vel)
        #self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Int32, self.callback_cmd_vel)
        
        #Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        #self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Int32, queue_size=1)
        #self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.v_r_pub = rospy.Publisher("/v_r", Int16, queue_size=1)
        self.v_l_pub = rospy.Publisher("/v_l", Int16, queue_size=1)

        self.command_string = String()
        self.command_int32 = Int32()
        self.command_twist = Twist()

        #Variables related to the odometry
        self.wheelDistance = 0.30
        self.odomA = 0
        self.odomA_previous = 0
        self.odomB = 0
        self.odomB_previous = 0
        self.odomCenter = 0
        self.distancePerCount = 0.0185
        self.wheelDiameter = 0.065
        self.odomX = 0
        self.odomY = 0 
        self.odomTh = 0
        self.odomQuat = Quaternion()
        self.odomTransform = TransformStamped()
        self.odom = Odometry()
        self.odomLastTime = rospy.get_rostime()

        #Variables related to cmd_vel
        self.v_r = Int16()
        self.v_l = Int16()

    def compute_odom(self):
        #print("compute_odom ----------------------------------")
        now = rospy.get_rostime()

        deltaRight = float(self.odomA - self.odomA_previous)
        deltaLeft = float(self.odomB - self.odomB_previous)
        self.odomA_previous = self.odomA
        self.odomB_previous = self.odomB
        #print("deltaRight is: " + str(deltaRight))
        #print("deltaLeft is: " + str(deltaLeft))

        dt = (now - self.odomLastTime).to_sec()
        #print("dt is: " + str(dt))

        omega_left = float(deltaLeft * self.distancePerCount) / dt
        omega_right = float(deltaRight * self.distancePerCount) / dt
        #print("omega_left is: " + str(omega_left))
        #print("omega_right is: " + str(omega_right))

        v_left = omega_left * self.wheelDiameter
        v_right = omega_right * self.wheelDiameter
        #print("v_left is: " + str(v_left))
        #print("v_rght is: " + str(v_right))

        vx = float((v_right + v_left)/2)
        vy = 0;
        vth = ((v_right - v_left)/ self.wheelDistance)

        deltaX = (vx * math.cos(vth)) * dt
        deltaY = (vx * math.sin(vth)) * dt
        deltaTh = vth * dt

        self.odomTh += deltaTh

        '''
        #OdomQuater is keep to self. or numpy array pb
        self.odomQuat = tf.transformations.quaternion_about_axis(self.odomTh, (0,0,1))
        print("odomQuat is: " + str(self.odomQuat))
        '''
        q = tf.transformations.quaternion_from_euler(0, 0, self.odomTh)
        self.odomQuat = Quaternion(*q)
        
        '''
        #TODO Send broacast transform
        self.odomTransform.header.stamp = now
        self.odomTransform.header.frame_id = "odom"
        self.odomTransform.child_frame_id = "base_link"

        self.odomTransform.transform.translation.x = self.odomX
        self.odomTransform.transform.translation.y = self.odomY
        self.odomTransform.transform.translation.z = 0
        self.odomTransform.transform.rotation = self.odomQuat
        '''

        self.odom.header.stamp = now
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.position.x += deltaX
        self.odom.pose.pose.position.y += deltaY
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = self.odomQuat

        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist.linear.x = vx
        self.odom.twist.twist.linear.y = vy
        self.odom.twist.twist.angular.z = vth

        self.odom_pub.publish(self.odom)
        #print("odom is: " +str(self.odom))

        self.odomLastTime = now


    def callback_chatter(self, msg):
        i = 1
        #print("inside callback_objects_adb")
        #print("This is the callback_chatter: " + str(msg.data))

    def callback_odomA(self, msg):
        self.odomA = msg.data
        #print("odomA is: " + str(msg.data))

    def callback_odomB(self, msg):
        self.odomB = msg.data
        #print("odomB is: " + str(msg.data))
        self.compute_odom()

    def callback_cmd_vel(self, msg):
        print("Inside callback_cmd_vel")
        print("self.command_twist is: " + str(self.command_twist))
        radian_per_sec_r = ((2 * msg.linear.x) + (self.wheelDistance * msg.angular.z) ) / (self.wheelDiameter)
        radian_per_sec_l = ((2 * msg.linear.x) - (self.wheelDistance * msg.angular.z) ) / (self.wheelDiameter)
        print("conversion_v_r is: " + str(radian_per_sec_r))
        print("conversion_v_l is: " + str(radian_per_sec_l))

        #Cross product radiant per second to 255
        self.v_r = abs(int((255 * radian_per_sec_r) / 7.84))
        self.v_l = abs(int((255 * radian_per_sec_l) / 7.84))

        if (self.v_l > 255):
            self.v_l = 255

        if (self.v_r > 255):
            self.v_r = 255

        if (msg.linear.x < 0):
            self.v_l += 1000
            self.v_r += 1000

        print("self.v_r is: " + str(self.v_r))
        print("self.v_l is: " + str(self.v_l))

        self.v_r_pub.publish(self.v_r)
        self.v_l_pub.publish(self.v_l)

        if(msg.linear.x == 0.0 and msg.angular.z == 0.0):
            self.shutdown_function()


        #Will do the order for 3 sec then will wait
        self.publish_cmd_vel_stop(3)


    def publish_cmd_vel_stop(self, wait_time):

        rospy.sleep(wait_time)

        self.command_twist.linear.x = 0.0
        self.command_twist.angular.z = 0.0
        self.v_r = 0
        self.v_l = 0
        self.v_r_pub.publish(self.v_r)
        self.v_l_pub.publish(self.v_l)
        self.cmd_vel_pub.publish(self.command_twist)


    def shutdown_function(self):
        #self.command_int32.data = 0
        self.command_twist.linear.x = 0.0
        self.command_twist.angular.z = 0.0
        self.v_r = 0
        self.v_l = 0
        self.v_r_pub.publish(self.v_r)
        self.v_l_pub.publish(self.v_l)
        self.cmd_vel_pub.publish(self.command_twist)
    
if __name__ == '__main__':
    rospy.init_node('billy_v0_node')
    messia = Robot()
    #messia.publish_cmd_vel()
    rospy.on_shutdown(messia.shutdown_function)

    while not rospy.is_shutdown():
        #messia.publish_cmd_vel()
        print(rospy.get_rostime())
        rospy.sleep(3)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass