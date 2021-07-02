### carla_ros_bridge 

- 节点：carla_ros_bridge(client.py)
- 订阅：向/carla_ackermann_control_ego_vehicle 订阅 carla/ego_vehicle/vehicle_control_cmd 消息
- 发布：/carla/ego_vehicle/vehicle_control_info
- 发布：/carla/ego_vehicle/vehicle_control_status
- 发布：/carla/ego_vehicle/vehicle_control_odometry.

##### Parent类

- parent.py (创建actor， 并管理子对象的生命周期)
- 初始化：carla_id = 0 ， 是桥对象，= -1，地图对象，>0 是actor对象。

##### CarlaRosBridge 类

- client.py 一个中 初始化了 CarlaRosBridge（或者BAG） 类（bridge.py），调用init 函数 和 run 函数。
- CarlaRosBridge 类继承自 Parent 类
- Parent 类是所有管理所有物体的源头；carla-ros-bridge 中类的继承关系如下图所示
- Parent -> bridge, child -> map, actor -> vehicle, sensor, traffic.
- Vehicle -> ego_vehicle
- Sensor -> lidar, camera -> rgb_camera, depth...
- 订阅：/carla/ego_vehicle/vehicle_control_cmd 话题； 
- 发布：/carla/ego_vehicle/vehicle_control_info
- 发布：/carla/ego_vehicle/vehicle_control_status
- 发布：/carla/ego_vehicle/vehicle_control_odometry.
- 以上有关车辆的发布和订阅均在 ego_vehicle 类里面，也就是ego_vehicle.py
- vehicle_control_odometry

- 

```
pose: 来自geometry_msgs的PoseWithCovariance,表示空间中含有不确定性的位姿信息
twist:来自geometry_msgs的TwistWithCovariance,消息定义了包含不确定性的速度量，协方差矩阵按行分别表示;沿x方向速度的不确定性，沿y方向速度的不确定性，沿z方向速度的不确定性;
      绕x转动角速度的不确定性，绕y轴转动的角速度的不确定性，绕z轴转动的角速度的不确定性;
```

```
odometry.pose: 
    1.odometry.pose.pose: 来自geometry_msgs 的Pose
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
    2.odometry.pose.civariance: 协方差
    
odometry.twist
    1.odometry.twist.twist: 类型是geometry_msgs的Twist()
        1.linear 类型：geometry_msgs的Vector3
            由carla_velocity转化而来,不转到angular
            1.x = carl_velocity.x
            2.y = -carl_velocity.y
            3.z = carl_velocity.z
        2.angular 类型：geometry_msgs的的Vector3
            1.x
            2.y
            3.z
    2.odometry.twist.civariance: 协方差
```

- vehicle_control_status(有用消息):

```
velocity:速度  
acceleration：加速度   
orientation:geometry_msgs/Quaternion （空间中代表旋转的四元数）  
carla_msgs: CarlaEgoVehicleControl类
（throttle油门，steer方向盘，brake刹车，hand_brake手刹,reverse倒车,gear档位 manual_gear_shift 手动换挡)
```

- vehicle_control_info

```
id: vehcile 的 actor_id  
type: actor.type_id blueprint id  
rolename: 角色名字
wheels: 轮胎
    tire_friction:表示轮胎的摩擦
    damping_rate:车轮的阻尼率
    steer_angle: 车轮转向的角度(已转化为弧度单位)
    disable_steering:最大转角
    ----以下info中不包含----
    max_brake_torque: 以Nm为单位的最大制动扭矩
    max_handbrake_torque：以Nm为单位的最大手制动扭矩
    location：位置（只读）
max_rpm: 车辆发动机的最大RPM（最大功率转速）
moi: 车辆发动机的惯性矩（车辆抵抗弯曲性质的能力）
damping_rate_full_throttle:  油门最大时的阻尼率
damping_rate)zero_throttle_clutch_engaged:  离合器踩上时油门为零的阻尼率
damping_rate)zero_throttle_clutch_disengaged:离合器松开时油门为零的阻尼率
use_gear_autobox:   true，车辆自动变速(变档)
gear_switch_time:   档位切换时机
clutch_strength:    车辆的离合器强度，以kg*m^2/s 测量
mass:   以kg为测量单位的车辆质量
drag_coefficient:  空气阻力系数
center_of_mass:    车辆的质心
----以下参数info类中不包含----
final_ratio:  从变速器到车轮的固定比率
GearPhysicsControl： 档位具体细节 (ratio、down_ratio,up_ratio)
```

- [max_rpm](https://car.autohome.com.cn/shuyu/detail_18_21_296.html)；[moi](https://baike.baidu.com/item/%E6%83%AF%E6%80%A7%E7%9F%A9)；

#### 这个库的运行逻辑是这样的:

1. 运行client.py，构建bridge对象，初始化过程中有super.init()也就是parent.init()，bridge初
   始化的时候订阅了world.on_tick(), 运行bridge.run()
2. bridge.run()中仅仅是ros::spin
3. 每一个tick会执行:锁线程，更新时钟并publish信息;运行bridge.update_child_actors->
   parent.update_child_actors, 更新actors。
4. 更新actor的过程:

- a. 记录下新生的actor,除掉已经被删除的actor,并且让所有的actor进行一次update() 
- b. 这里进行了改写，属于Sensor的actor不会由parent.child_actor管理，而由
  sensor_manager管理，sensor_manager.update()会让属于Sensor的所有actor进
  行update.
- c. 在记录新生Actor,除掉旧actor过程中，sensor_manager也如上文所说接管了
  sensor类的所有actor.

5. 记录新生的actor时，会创建一个对应的sensor对象，比如说carla仿真中的一个rgbcamera
   会使得sensor_manager创建一个ros_bridge中的rgb_camera对象，这个rgb_camera对象 的初始化会一步一步往上继承，在sensor类提供的init()中注册了一个listen函数; 这个listen 的订阅对应的函数_callback_sensor_data .里面调用sensor_data_updated负责将carla的 信息转化为msg信息，并调用camera或lidar的self.publish_ros_message;
6. 传感器对象的这 个publish_ros_msg函数首先继承自child，而child则调用建立child过程中,输入的parent的 publish_ros_message;在parent.py 的新生actor语句中，我们建立sensor对象写的是 parent=self, 这个self继承了整个update、create_new方法的bridge对象，(尽管还有别的 路径，不过最终)最后会跑bridge.publish_ros_message，这个函数将ROS信息存到了输 出缓存中，在tick的订阅函数的最后一步输出。
7. 在bridge 的 tick 订阅函数 self._carla_update_child_actors 函数中进一步往下寻找，-> self.update()->actor.update()->调用了vehicle.update()中发送消息。 self._prepare_msgs() 调用了 bridge.publish_ros_message() 函数，这个函数的输入是 物体 的消息，将其添加至缓存信息中，在send_msgs()函数中进一步输出。