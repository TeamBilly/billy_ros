#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Vector3
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped


class HeadControlRobot:
    def __init__(self):
        rospy.init_node('rosserial_control_robot')
        self.joy_sub = rospy.Subscriber('/headsetPose', PoseStamped, self.headset_callback)
        self.rotation_pub = rospy.Publisher('/headset_Rotation', Vector3, queue_size=1)
        self.robotOrientation = Vector3()
        

    def headset_callback(self, msg):
        (self.robotOrientation.x, self.robotOrientation.y, self.robotOrientation.z) = euler_from_quaternion([msg.pose.orientation.x, 
                 msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], 
                 'sxyz')
        
        self.robotOrientation.x = degrees(self.robotOrientation.x)
        self.robotOrientation.y = - degrees(self.robotOrientation.y)
        self.robotOrientation.z = degrees(self.robotOrientation.z)
        print('x = ' + str(self.robotOrientation.x) +"y = "+ str(self.robotOrientation.y) +"z = " + str(self.robotOrientation.z))
        self.rotation_pub.publish(self.robotOrientation)

if __name__ == '__main__':
    HeadControlRobot()
    rospy.spin()