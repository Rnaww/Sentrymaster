#pragma once
#include "common.h"
#include <nav_msgs/msg/path.hpp>

class PlanningCore;
class TerrainSpeed;
#include "rm_msgs/msg/navigation_status.hpp"

class SentryPlannerNode : public rclcpp::Node {
public:
    SentryPlannerNode(std::string name);

private:
    using StatusMsg = rm_msgs::msg::NavigationStatus;
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
    rclcpp::Publisher<StatusMsg>::SharedPtr status_puber;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_puber;
    rclcpp::TimerBase::SharedPtr control_timer;
    DecisionNavMsg decision{};
    OdomMsg odom{};
    ChassisDataMsg chassis{};
    PcdMsg::SharedPtr cloud;
    Eigen::Vector3d current_odom_pos_2d = Eigen::Vector3d::Zero();
    double yaw = 0, last_odom_time = -1, last_command_time = -1;
    double odom_timeout, command_timeout;
    uint8_t last_state = 255;
    void OdomCallBack(OdomMsg::SharedPtr msg);
    void ChassisInfoCallback(ChassisDataMsg::SharedPtr msg);
    void DecisionCallback(DecisionNavMsg::SharedPtr msg);
    void CloudCallback(PcdMsg::SharedPtr msg);
    void control_cbk();
    void pub_status(uint8_t state, const std::string &reason);
};
