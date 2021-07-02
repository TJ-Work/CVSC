### Ramcontrol2carla.py

- **综述：** rambot控制器输出的twist 与carla不兼容，此节点实现rambot_controller 发布的twist的转换  
- **节点：** Ramcontrol2carla  
- **订阅：** 名称为/RobotPort_CmdVel (来自rambot_controller) 中的Twist消息 
- **发布：** 名称为/carla/ego_vehicle/ackermann_cmd 的Twist 消息
- ros中机器人的移动一般由 twist中 linear的x方向 的线速度 和 angular中的z 轴角速度所决定，其余方向的速度不存在。   
- 有关ros中 geometry_msgs类的请参考：http://wiki.ros.org/geometry_msgs   
- ros中 twist 解释： https://www.cnblogs.com/shang-slam/p/6891086.html
- ackermann_control 中控制命令主要由 速度（speed） 和 打角（steering_angle）构成

## carla_global_twist2local

综述：odom从ros桥出来，经过这个适应器才发往控制器
local_odometry: 
订阅：/carla/ego_vehicle/odometry
发布：/RobotPort_Odom

```
local_odometry.pose: (没有变化)
    1.local_odometry.pose.pose: 来自geometry_msgs 的Pose
        1. position : 类型是geometry_msgs 的 Point
            由carla.transform.location 类转化而来，该转化
            直接赋值即可。变量类型一样。具体请参见tdransforms.py.
            1.x
            2.y
            3.z
        2. orientation:类型是自geometry_msgs的Quaternion
            由carla.transform.rotation类转化而来,转化方式如下
            一、先将欧拉角转换为弧度制；
            二、调用python的api转换为quaternion；
            三、geometry_msgs.Quatiernion中的x,y,xw,分别对应四元数的四位
            1. x
            2. y
            3. z
            4. w
    2.local_odometry.pose.civariance: 协方差
    
local_odometry.twist
    1.loacal_odometry.twist.twist: 类型是geometry_msgs的Twist()
        1.linear 类型：
            1.x = sqrt(x*x + y*y +z*z)
            2.y = 0
            3.z = 0
        2.angular 类型：geometry_msgs的的Vector3
            1.x
            2.y
            3.z = z(? rosbridge 好像没有给出来)
    2.local_odometry.twist.civariance: 协方差
```

## local_path_adapter

- 综述：订阅local_waypoint_publischer发布的local_path,并调整格式使之适应rambot_controller
- 订阅：/carla/ego_vehicle/local_target_path,回调函数中发布一个DesiredTrajectory 类，中间包括重要信息为：一个位姿信息poses的list；culvature 曲率list；speeds 速度数组；其中速度默认保持恒速，为3m/s，culvature 为0 list；
- 发布：名称为RobotPort_Cmd的DesiredTrajectory消息；

## indicator_adapter

- 综述： 让显示器适应到正确的frame
- 订阅：名字/carla/steering_indicator的 Marker消息，ros中显示在rviz中的消息。
- 订阅：/carla/speed_indicator Marker 消息
- 订阅：/carla/Controller_fitted_path 的Path 消息
- 发布相应的 /steering_indicator;/speed_indicator; /Controller_fitted_path,消息
- [marker消息连接](http://wiki.ros.org/rviz/DisplayTypes/Marker) ； [补充](https://blog.csdn.net/wilylcyu/article/details/57080917)
- 回调操作：将 接收到的msg消息header换成map，rviz的显示需要一个世界坐标系，这里使用的map，然后再重新发布，最终在rviz的配置文件中，rviz 订阅了这些话题，并完成显示。

## 