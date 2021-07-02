#!/usr/bin/env python
"""
        get indicators into correct frames
"""
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Header
from math import *

rospy.init_node("indicator_adapter", anonymous=False)
message_pub1 = rospy.Publisher("/carla/steering_indicator", Marker, queue_size=1)
message_pub2 = rospy.Publisher("/carla/speed_indicator", Marker, queue_size=1)
message_pub3 = rospy.Publisher("/carla/Controller_fitted_path", Path, queue_size=1)


def rambot_steer_callback(msg):
    global message_pub1
    message = msg
    message.header = Header(frame_id="map")
    message_pub1.publish(message)

def rambot_speed_callback(msg):
    global message_pub2
    message = msg
    message.header = Header(frame_id="map")
    message_pub2.publish(message)

def rambot_path_callback(msg):
    global message_pub3
    message = msg
    message.header = Header(frame_id="map")
    for poseStamped in message.poses:
        poseStamped.header = Header(frame_id="map")
    message_pub3.publish(message)


rospy.Subscriber("/steering_indicator", Marker, rambot_steer_callback)
rospy.Subscriber("/speed_indicator", Marker, rambot_speed_callback)
rospy.Subscriber("/RobotPort_CpathFitted", Path, rambot_path_callback)

rospy.spin()
