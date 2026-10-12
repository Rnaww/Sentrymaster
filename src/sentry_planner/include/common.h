#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/util/Console.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <thread>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <queue>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
using PcdMsg = sensor_msgs::msg::PointCloud2;

#include <std_msgs/msg/bool.hpp>
using BoolMsg = std_msgs::msg::Bool;

#include <nav_msgs/msg/odometry.hpp>
using OdomMsg = nav_msgs::msg::Odometry;

#include "rm_msgs/msg/decision_nav_cmd.hpp"
using DecisionNavMsg = rm_msgs::msg::DecisionNavCmd;

#include "rm_msgs/msg/chassis_data.hpp"
using ChassisDataMsg = rm_msgs::msg::ChassisData;

#include "rm_msgs/msg/navigation_mcu_cmd.hpp"
using ChassisControlMsg = rm_msgs::msg::NavigationMcuCmd;

inline void info_log() {std::cout << "\33[0m" << std::endl;}
// 发布绿色信息
template<typename T, typename... Args>
void info_log(T element, Args... args) {
    std::cout << "\33[32m" << element << " "; 
    info_log(args...);         
}

inline void error_log() {std::cout << "\33[0m" << std::endl;}
// 发布红色信息
template<typename T, typename... Args>
void error_log(T element, Args... args) {
    std::cout << "\33[31m" << element << " "; 
    error_log(args...);         
}

enum SentryPosture{
    ATTACK = 1,
    DEFENSE = 2,
    MOVE = 3
};
