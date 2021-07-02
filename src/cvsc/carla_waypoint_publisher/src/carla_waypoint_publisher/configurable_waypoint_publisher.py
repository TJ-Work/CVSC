#!/usr/bin/env python
import rospy
import carla
from carla_waypoint_publisher import CarlaToRosWaypointConverter

class ConfigurableWayPointConverter(CarlaToRosWaypointConverter):
    def __init__(self, carla_world):
        super(ConfigurableWayPointConverter, self).__init__(carla_world)
        self.goal_index = int(rospy.get_param("~goal_index", '0'))
        self.goal_pose = rospy.get_param("~goal_pose", "")
        pose_set = False
        if len(self.goal_pose) > 10:
            self.goal_pose = self.goal_pose.split(',')
            if len(spawn_point) == 6:
                carla_goal = carla.Transform()
                carla_goal.location.x = float(self.goal_pose[0])
                carla_goal.location.y = -float(self.goal_pose[1])
                carla_goal.location.z = float(self.goal_pose[2]) + 2  # 2m above ground
                yaw = float(self.goal_pose[5])
                carla_goal.rotation.yaw = -math.degrees(yaw)
        if pose_set:
            self.goal = carla_goal
        else:
            self.goal = self.world.get_map().get_spawn_points()[self.goal_index]

def main():
    """
    main function
    """
    rospy.init_node("carla_waypoint_publisher", anonymous=True)

    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        waypointConverter = ConfigurableWayPointConverter(carla_world)

        rospy.spin()
        del waypointConverter
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()