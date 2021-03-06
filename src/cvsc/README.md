## Note for Unity Drive Developers:

The package is divided into several packages and almost all have individual ReadMe.md and launch files.

    1. carla_ros_bridge        : Basiced package for communicating with ROS and Carla.
    2. carla_ego_vehicle       : Package for defining (sensors, car_type) and spawning the "main" vehicle.
    3. carla_waypoint_publisher: Package providing global and local plan.
    4. carla_manual_control    : Official package for pygame manual control.
    5. carla_ackermann_control : Official controller to convert "Twist" command into throttle, brake, steering used in Carla.
    6. carla_msgs              : Message Package used "between" ros_bridge packages, don't have to be exposed. 
    7. carla_rambot_adapters   : Customized Package for adapting topics towards rambot.
    8. carla_launcher          : Package for launching the entire simulations with multiple packages.

<table>
  <tr>
    <td bgcolor=#98FB98><font color="#FFFFFF"></font>！Important</font></td>
  </tr>
  <tr>
    <td bgcolor=#F0FFF0><font color="#FFFFFF"></font>we have already make a special docs for carla_Unity_Drive control Developers, please try the link below! </font></td>
  </tr>
</table>

Here is the ***[docs_for_Unitydrive_controller](./docs/readme.md)***;

Or the page on HTML style: ***[docs_for_Unitydrive_controller](./site/readme.html)***;

There is an official comprehensive launcher for manual control, ego_vehicle, ros_bridge, just read forward.

# ROS bridge for CARLA simulator

This ROS package aims at providing a simple ROS bridge for CARLA simulator.

__Important Note:__
This documentation is for CARLA versions *newer* than 0.9.4.

![rviz setup](./docs/images/rviz_carla_default.png "rviz")
![depthcloud](./docs/images/depth_cloud_and_lidar.png "depthcloud")

![short video](https://youtu.be/S_NoN2GBtdY)


# Features

- [x] Cameras (depth, segmentation, rgb) support
- [x] Transform publications
- [x] Manual control using ackermann msg
- [x] Handle ROS dependencies
- [x] Marker/bounding box messages for cars/pedestrian
- [x] Lidar sensor support
- [ ] Rosbag in the bridge (in order to avoid rosbag record -a small time errors)
- [ ] Add traffic light support

# Setup

## Create a catkin workspace and install carla_ros_bridge package

    #setup folder structure
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
    cd ~/carla-ros-bridge
    git clone https://github.com/carla-simulator/ros-bridge.git
    cd catkin_ws/src
    ln -s ../../ros-bridge
    source /opt/ros/kinetic/setup.bash
    cd ..
    
    #install required ros-dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src -r
    
    #build
    catkin_make

For more information about configuring a ROS environment see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

# Start the ROS bridge

First run the simulator (see carla documentation: http://carla.readthedocs.io/en/latest/)

    ./CarlaUE4.sh -windowed -ResX=320 -ResY=240 -benchmark -fps=10


Wait for the message:

    Waiting for the client to connect...

Then start the ros bridge (choose one option):

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
    
    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge carla_ros_bridge.launch
    
    # Option 2: start the ros bridge together with RVIZ
    roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch
    
    # Option 3: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

You can setup the ros bridge configuration [carla_ros_bridge/config/settings.yaml](carla_ros_bridge/config/settings.yaml).

As we have not spawned any vehicle and have not added any sensors in our carla world there would not be any stream of data yet.

You can make use of the CARLA Python API script manual_control.py.
```
cd <path/to/carla/>
python manual_control.py --rolename=ego_vehicle
```
This spawns a carla client with role_name='ego_vehicle'. 
If the rolename is within the list specified by ROS parameter `/carla/ego_vehicle/rolename`, the client is interpreted as an controllable ego vehicle and all relevant ROS topics are created.

To simulate traffic, you can spawn automatically moving vehicles by using spawn_npc.py from CARLA Python API.

# Available ROS Topics

## Ego Vehicle

### Odometry

|Topic                          | Type |
|-------------------------------|------|
| `/carla/<ROLE NAME>/odometry` | [nav_msgs.Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) |

### Sensors

The ego vehicle sensors are provided via topics with prefix /carla/ego_vehicle/<sensor_topic>

Currently the following sensors are supported:

#### Camera

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/camera/rgb/<SENSOR ROLE NAME>/image_color` | [sensor_msgs.Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) |
| `/carla/<ROLE NAME>/camera/rgb/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs.CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) |

#### Lidar

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/lidar/<SENSOR ROLE NAME>/point_cloud` | [sensor_msgs.PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) |

#### GNSS

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/gnss/front/gnss` | [sensor_msgs.NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) |

#### Collision Sensor

|Topic                          | Type |
|-------------------------------|------|
| `/carla/<ROLE NAME>/collision` | [carla_msgs.CarlaCollisionEvent](carla_msgs/msg/CarlaCollisionEvent.msg) |

#### Lane Invasion Sensor

|Topic                          | Type |
|-------------------------------|------|
| `/carla/<ROLE NAME>/lane_invasion` | [carla_msgs.CarlaLaneInvasionEvent](carla_msgs/msg/CarlaLaneInvasionEvent.msg) |

### Object Sensor

|Topic         | Type |
|--------------|------|
| `/carla/<ROLE NAME>/objects` | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) |

Reports all vehicles, except the ego vehicle.

### Control

|Topic                                 | Type |
|--------------------------------------|------|
| `/carla/<ROLE NAME>/vehicle_control_cmd` (subscriber) | [carla_msgs.CarlaEgoVehicleControl](carla_msgs/msg/CarlaEgoVehicleControl.msg) |
| `/carla/<ROLE NAME>/vehicle_control_cmd_manual` (subscriber) | [carla_msgs.CarlaEgoVehicleControl](carla_msgs/msg/CarlaEgoVehicleControl.msg) |
| `/carla/<ROLE NAME>/vehicle_control_manual_override` (subscriber) | [std_msgs.Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html) |
| `/carla/<ROLE NAME>/vehicle_status` | [carla_msgs.CarlaEgoVehicleStatus](carla_msgs/msg/CarlaEgoVehicleStatus.msg) |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs.CarlaEgoVehicleInfo](carla_msgs/msg/CarlaEgoVehicleInfo.msg) |

There are two modes to control the vehicle.

1. Normal Mode (reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd`)
1. Manual Mode (reading commands from `/carla/<ROLE NAME>/vehicle_control_cmd_manual`)

This allows to manually override a Vehicle Control Commands published by a software stack. You can toggle between the two modes by publishing to `/carla/<ROLE NAME>/vehicle_control_manual_override`.

[carla_manual_control](carla_manual_control/) makes use of this feature.


For testing purposes, you can stear the ego vehicle from the commandline by publishing to the topic `/carla/<ROLE NAME>/vehicle_control_cmd`.

Examples for a ego vehicle with role_name 'ego_vehicle':

Max forward throttle:

     rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 0.0}" -r 10


Max forward throttle with max steering to the right:

     rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10


The current status of the vehicle can be received via topic `/carla/<ROLE NAME>/vehicle_status`.
Static information about the vehicle can be received via `/carla/<ROLE NAME>/vehicle_info`

#### Carla Ackermann Control

In certain cases, the [Carla Control Command](carla_ros_bridge/msg/CarlaEgoVehicleControl.msg) is not ideal to connect to an AD stack.
Therefore a ROS-based node ```carla_ackermann_control``` is provided which reads [AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html) messages.
You can find further documentation [here](carla_ackermann_control/README.md).


## Other Topics

### Object information of all vehicles

|Topic         | Type |
|--------------|------|
| `/carla/objects` | [derived_object_msgs.ObjectArray](http://docs.ros.org/api/derived_object_msgs/html/msg/ObjectArray.html) |
| `/carla/vehicle_marker` | [visualization_msgs.Maker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html) |


## Map

|Topic         | Type |
|--------------|------|
| `/carla/map` | [std_msgs.String](http://docs.ros.org/api/std_msgs/html/msg/String.html) |

The OPEN Drive map description is published.


# Carla Ego Vehicle

`carla_ego_vehicle` provides a generic way to spawn an ego vehicle and attach sensors to it. You can find further documentation [here](carla_ego_vehicle/README.md).


# Waypoint calculation

To make use of the Carla waypoint calculation a ROS Node is available to get waypoints. You can find further documentation [here](carla_waypoint_publisher/README.md).


# ROSBAG recording (not yet tested)

The carla_ros_bridge could also be used to record all published topics into a rosbag:

    roslaunch carla_ros_bridge client_with_rviz.launch rosbag_fname:=/tmp/save_session.bag

This command will create a rosbag /tmp/save_session.bag

You can of course also use rosbag record to do the same, but using the ros_bridge to do the recording you have the guarentee that all the message are saved without small desynchronization that could occurs when using *rosbag record* in an other process.


# Troubleshooting

## ImportError: No module named carla

You're missing Carla Python. Please execute:

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>

Please note that you have to put in the complete path to the egg-file including
the egg-file itself. Please use the one, that is supported by your Python version.
Depending on the type of CARLA (pre-build, or build from source), the egg files
are typically located either directly in the PythonAPI folder or in PythonAPI/dist.

Check the installation is successfull by trying to import carla from python:

    python -c 'import carla;print("Success")'

You should see the Success message without any errors.
