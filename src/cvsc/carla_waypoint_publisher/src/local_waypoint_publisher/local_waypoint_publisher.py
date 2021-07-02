#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class LocalWaypoingNode:
    def __init__(self):
        rospy.init_node("ros_node")
        rospy.loginfo("Starting LocalWaypoingNode.")

        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self.max_local_point_num = int(
            rospy.get_param("~local_point_num", "10"))
        assert self.max_local_point_num > 0

        self.global_path = None  # list of geometry_msgs::pose
        self.index = None

        self.global_waypoints_subscriber = rospy.Subscriber(
            "/carla/{}/waypoints".format(self.role_name), Path, self.waypoint_callback)
        self.odom_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(self.role_name), Odometry, self.odom_callback)
        self.local_point_publisher = rospy.Publisher(
            "/carla/{}/local_target_path".format(self.role_name), Path, queue_size=1)

    def waypoint_callback(self, msg):
        self.index = 0
        pose_stamps = msg.poses
        self.global_path = []
        for pose_stamp in pose_stamps:
            self.global_path.append(pose_stamp.pose)

    def odom_callback(self, msg):
        if self.global_path is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self._update_index(x, y, z)
        self._publish()

    def _publish(self):
        new_msg = Path()
        new_msg.header.frame_id = "map"
        new_msg.poses = []
        temp_index = self.index

        while temp_index < len(self.global_path) and len(new_msg.poses) < self.max_local_point_num:
            pose = PoseStamped()
            pose.pose = self.global_path[temp_index]
            new_msg.poses.append(pose)

            temp_index += 1

        self.local_point_publisher.publish(new_msg)

    # def _update_index(self, x, y):
    #     indexes = [self.index - 1, self.index, self.index + 1, self.index + 2]
    #     distances = []
    #     for index in indexes:
    #         if 0 <= index < len(self.global_path):
    #             pose = self.global_path[index]
    #             x1 = pose.position.x
    #             y1 = pose.position.y
    #             distances.append(
    #                 (x1 - x) ** 2 + (y1 - y) ** 2
    #             )
    #         else:
    #             distances.append(1e10)
    #     self.index = indexes[np.argmin(distances)]

    def _update_index(self, x, y, z):
        get_next = True
        while get_next:
            get_next = False
            
            if self.index == len(self.global_path) - 1:
                return

            else:
                this_point = self.global_path[self.index]
                next_point = self.global_path[self.index + 1]
                this_to_car = np.array(
                    [x - this_point.position.x,
                    y - this_point.position.y,
                    z - this_point.position.z]
                )
                this_to_next = np.array(
                    [next_point.position.x - this_point.position.x,
                    next_point.position.y - this_point.position.y,
                    next_point.position.z - this_point.position.z]
                )
                distance_norm = np.sum(np.square(this_to_next))
                if this_to_car.dot(this_to_next) >= np.sum(np.square(this_to_next)) or distance_norm < 0.3:
                    self.index += 1
                    get_next = True

if __name__ == "__main__":
    ros_node = LocalWaypoingNode()
    rospy.spin()
