#!/usr/bin/env python
"""
        the odom carla_ros_bridge output with twist in global frame 
        adapt this twist to local
        Here is the adapter between rambot_controller and carla_ros_bridge
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import *

rospy.init_node("TwistAdapter", anonymous=False)
message_pub = rospy.Publisher("RobotPort_Odom", Odometry, queue_size=1)

def rambot_odom_callback(msg):
    global message_pub
    speed = sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2 + msg.twist.twist.linear.z ** 2)
    message = Odometry()
    message.pose = msg.pose
    message.twist.twist.linear.x = speed
    message.twist.twist.angular.z = msg.twist.twist.angular.z
    message_pub.publish(message)

rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, rambot_odom_callback)

rospy.spin()
