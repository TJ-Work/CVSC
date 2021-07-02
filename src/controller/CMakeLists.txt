cmake_minimum_required(VERSION 2.8.3)
project(controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

catkin_package()

find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
)

include_directories(
  ../
  ./
  ${PROJECT_NAME}/../
  ${catkin_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(	rambot_controller
    
    
)

target_link_libraries(controller 
                      ${catkin_LIBRARIES} 
                      # ${Boost_LIBRARIES} 
                      ${GLOG_LIBRARIES}
                      control_type_proto
                      proto
                      glog
		      qpOASES
                      )

add_dependencies(controller ${PROJECT_NAME}_gencfg)

add_subdirectory(src/proto)
add_subdirectory(src/data_types)

install(
  DIRECTORY 
  launch
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY 
  src/param
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  TARGETS 
  controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

