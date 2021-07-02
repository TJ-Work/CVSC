#!/usr/bin/env python
"""
        rambot_controller package output twist command in an unusual way.
        Here is the adapter between rambot_controller and carla_ros_bridge
"""
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

rospy.init_node("RamControl2Carla", anonymous=False)
message_pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, queue_size=1)

def rambot_twist_callback(msg):
    global message_pub
    speed = msg.linear.x
    steering_angle = msg.angular.z
    message = AckermannDrive()
    message.steering_angle = steering_angle
    message.speed = speed
    message_pub.publish(message)

rospy.Subscriber("/RobotPort_CmdVel", Twist, rambot_twist_callback)

rospy.spin()





