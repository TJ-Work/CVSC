#!/usr/bin/env python
"""
    Local path are published in message type nav_msgs::Path
    This node converts this into rambot_msgs::DesiredTrajectory
"""
import rospy 
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from rambot_msgs.msg import DesiredTrajectory
from carla_msgs.msg import CarlaEgoVehicleStatus
from adapter_math.pose_math import curvature_from_poses, dist_of_poses

class RosNode:
    def __init__(self):
        rospy.init_node("ros_node")
        rospy.loginfo("Starting RosNode.")
        self.target_speed = float(rospy.get_param("~target_speed", "5"))
        self.controller_id = float(rospy.get_param("~controller_id", "2") )
        rospy.Subscriber("/carla/ego_vehicle/local_target_path", Path, self.path_callback)
        self.rambot_path_pub = rospy.Publisher("RobotPort_Cmd", DesiredTrajectory, queue_size=1)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.publish_target_speed)
        self.target_vehicle_status_pub = rospy.Publisher("target_speed",CarlaEgoVehicleStatus,queue_size= 1)
    def publish_target_speed(self, msg):
        target_vehicleStatus = CarlaEgoVehicleStatus()
        target_vehicleStatus.velocity = self.target_speed
        self.target_vehicle_status_pub.publish(target_vehicleStatus)

    def path_callback(self, msg):

        poses = []
        if len(msg.poses) <= 1:
            target_speed = 0
            target_culvature = -1
            behavior_type = 1
        else:
            target_speed = self.target_speed
            target_culvature = 0
            behavior_type = 0

        for i in range(len(msg.poses)):
            if i >= 1:
                distance = dist_of_poses(msg.poses[i-1].pose, msg.poses[i].pose)
                if distance < 0.1:
                    continue
            poses.append(msg.poses[i].pose)

        speed_list = [target_speed] * len(poses)

        if behavior_type == 1:
            culvatures = [target_culvature] * len(poses)
        else:
            culvatures = [abs(curvature_from_poses(poses[i], poses[i+1])) for i in range(len(poses) - 1)]
            culvatures += [culvatures[-1]]

        cmd = DesiredTrajectory(behavior_type=behavior_type, controller_id=self.controller_id,
                                poses=poses, culvatures=culvatures, speeds=speed_list)

        self.rambot_path_pub.publish(cmd)


if __name__ == "__main__":
    ros_node = RosNode()
    rospy.spin()
