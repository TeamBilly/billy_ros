#! /usr/bin/env python

import rospy
import tf
import math
import numpy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from sensor_msgs.msg import Image

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
        self.sub_cam = rospy.Subscriber('/rms/camera1/image_raw', Image, self.callback_cam)

        #Publishers
        self.pub_joint1 = rospy.Publisher("/rms/joint1_position_controller/command", Float64, queue_size=1)
        self.pub_joint2 = rospy.Publisher("/rms/joint2_position_controller/command", Float64, queue_size=1)
        self.image_pub = rospy.Publisher("/image_processed",Image, queue_size=1)

        self.joint1 = Float64()
        self.joint2 = Float64()

        #Flag and counter
        self.j1_flag = 0
        self.j2_flag = 0
        self.last_item = -1

        #Robot spec
        self.L1 = 0.1
        self.L2 = 0.1

        #Camera
        self.image_raw_sub = Image()

    def callback_cam(self, image_raw):
        self.image_raw_sub = image_raw

    def wave(self):
        #----------------joint1----------------
        if(abs(self.joint1.data) >= 3.0):
            if(self.j1_flag == 0):
                self.j1_flag = 1
            else:
                self.j1_flag = 0

        if(self.j1_flag == 1):
            self.joint1.data += 0.1
        else:
            self.joint1.data -= 0.1
        self.pub_joint1.publish(self.joint1)

        #----------------joint2----------------
        if(self.joint2.data == -1.0):
            self.joint2.data = -1.8
        else:
            self.joint2.data = -1.0
        self.pub_joint2.publish(self.joint2)


    def zero(self):
        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)

    def shutdown_function(self):
        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)
    
if __name__ == '__main__':
    rospy.init_node('scara_billy_node')
    scara_billy = Robot()
    scara_billy.zero()
    rospy.sleep(2)
    rospy.on_shutdown(scara_billy.shutdown_function)
    
    while not rospy.is_shutdown():
        rospy.sleep(2)
        scara_billy.wave()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
