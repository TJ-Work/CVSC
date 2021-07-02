

### carla_ackermann_control_ego_vehicle.py(有待改进)
- 综述：该ROS节点将[AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html)消息转换为CarlaEgoVehicleControl。该节点使用了pid 控制。
- PID控制器用于控制加速度/速度。
- 读取Carla控制所需的车辆信息（通过carla ros bridge） 
```
依赖：install python simple-pid 
      pip install --user simple-pi
```
- 更多请参考[官网链接](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ackermann_control)
- 节点：/carla_ackermann_control_ego_vehicle
- 订阅1：向 /Ramcontrol2carla 订阅 名称为 /carla/ego_vehicle/ackermann_cmd 的 AckermannDrive 类消息
- 订阅2：向 /carla_ros_bridge 订阅 名称为 /carla/ego_vehicle/vehicle_status 的  CarlaEgoVehicleStatus 消息（订阅这个有啥用？）
- 发布：向 /carla_ros_bridge 发布 名称为 /carla/ego_vehicle/vehicle_control_cmd 的 CarlaEgoVehicleControl 消息
- 初始化订阅3：/carla/ego_vehicle/vehicle_info
- 初始化发布2：/carla/ego_vehicle/ackermann_control/control_info

## scenario runner
- 综述：scenario runner 原本是carla 官方给出的一个 场景测试的 python 模块以及脚本。 其中本来已包含了ros 代理，本项目将其与 官方的ros 桥结合，以方便 **自定义** 路线功能。
- scenario runner 在本控制器项目中作为路线自定义的接口，以及返回对场景peformance的评价。
- 订阅:rosbridge中的/clock
- 发布:/local_way_point
- 发布在定义的control_assessment.py中发布路线，消息格式为：Path，在path里是一个poses[]数组，PoseStamped()下的pose 为 geometry_msgs 的 Pose，其中的pose.location 中的x，y，z由waypoints赋值给他。最后将所有的pose append 至数组。得到path
- 创建了一个control_assessment的场景后，该场景继承自basic_scenario,这个基础类初始化，创建了 behavior，已经critiria，最后在初始化的最后，开始调用setup，使其发布。

## local_waypoint_publischer
- 综述：通过waypoints和odometry 信息得出local_target_path
- 订阅：/carla/ego_vehicle/waypoints;回调函数中将path中所有的pose添加至global_path.
- 订阅：/carla/ego_vehicle/Odometry; 回调函数中更新了当前所处的waypoints的点，并在当前的waypoints往后取max_local_point_num(10)也就是最大的本地路径点，合成一个新的Path 消息publish出去
- 发布：发布一个新的Path消息（如上）



