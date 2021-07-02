#!/usr/bin/env python
"""
   some useful message will be handled here,in order to fit the plot;
"""
import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from rambot_msgs.msg import DesiredTrajectory
from carla_msgs.msg import CarlaEgoVehicleStatus 

class Handle_message:
    self.set_locations = []
    self.actual_locatios =[]

    def __init__(self):
        rospy.init_node("Handle_message")
        rospy.Subscriber("/RobotPort_CpathFitted", Path, self.path_callback)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.publish_trajectory_error) 
        self.target_vehicle_status_pub = rospy.Publisher("Trajectory error", Odometry, queue_size= 1)

    def publish_target_speed(self, msg):
        location = Path().poses[0].pose.location
        self.set_locations.append(location)
        if !len(self.locations):

            
        target_vehicleStatus = CarlaEgoVehicleStatus()
        target_vehicleStatus.velocity = self.target_speed
        self.target_vehicle_status_pub.publish(target_vehicleStatus)
        if len(msg.poses) <= 1:
            target_speed = 0
            target_culvature = -1
            behavior_type = 1
        else:
            target_speed = self.target_speed
            target_culvature = 0
            behavior_type = 0

        for i in range(len(msg.poses)):
            poses.append(msg.poses[i].pose)
        
        speed_list = [target_speed] * len(msg.poses)
        culvatures = [target_culvature] * len(msg.poses)

        cmd = DesiredTrajectory(behavior_type=behavior_type, controller_id=self.controller_id,
                                poses=poses, culvatures=culvatures, speeds=speed_list)

        self.rambot_path_pub.publish(cmd)

    def path_callback(self, msg):
        location = Path().poses[0].pose.location
        location.x = msg.poses[0].pose.location.x
        location.y = msg.poses[0].pose.location.y
        location z = msg.poses[0].pose.location.z
        self.set_locations.append(location)

if __name__ == "__main__":
    ros_node = RosNode()
    rospy.spin()