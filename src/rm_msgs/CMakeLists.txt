cmake_minimum_required(VERSION 3.8)
project(rm_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisData.msg"
  "msg/RefereeData.msg"
  "msg/DecisionNavCmd.msg"
  "msg/NavigationMcuCmd.msg"
  "msg/DecisionMcuCmd.msg"

  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

ament_package()
