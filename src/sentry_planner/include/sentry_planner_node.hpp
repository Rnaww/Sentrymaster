#pragma once
#include "common.h"
#include <nav_msgs/msg/path.hpp>

class PlanningCore;
class TerrainSpeed;

class SentryPlannerNode : public rclcpp::Node {
public:
    SentryPlannerNode(std::string name);

private:
    std::shared_ptr<PlanningCore> planning_core;
    std::shared_ptr<TerrainSpeed> terrain_speed;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_puber;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_suber;
    rclcpp::Subscription<ChassisDataMsg>::SharedPtr chassis_data_suber;
    rclcpp::Subscription<DecisionNavMsg>::SharedPtr decision_suber;
    rclcpp::Subscription<PcdMsg>::SharedPtr cloud_suber;
    rclcpp::Publisher<ChassisControlMsg>::SharedPtr navigation_puber;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_puber;
    rclcpp::TimerBase::SharedPtr control_timer;
    DecisionNavMsg decision{};
    OdomMsg odom{};
    ChassisDataMsg chassis{};
    PcdMsg::SharedPtr cloud;
    Eigen::Vector3d current_odom_pos_2d = Eigen::Vector3d::Zero();
    double yaw = 0, last_odom_time = -1, last_command_time = -1;
    double odom_stamp = -1, yaw_rate = 0;
    double odom_timeout, command_timeout;
    std::string last_reason;
    void OdomCallBack(OdomMsg::SharedPtr msg);
    void ChassisInfoCallback(ChassisDataMsg::SharedPtr msg);
    void DecisionCallback(DecisionNavMsg::SharedPtr msg);
    void CloudCallback(PcdMsg::SharedPtr msg);
    void control_cbk();
    void log_state(const std::string &reason);
};
