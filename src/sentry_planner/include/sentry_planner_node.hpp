#pragma once
#include "common.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include "rm_msgs/msg/navigation_status.hpp"
#include <nav_msgs/msg/path.hpp>

class TerrainSpeed;

class SentryPlannerNode : public rclcpp::Node{
public:
    SentryPlannerNode(std::string name);
private:
    using Navigate = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
    using StatusMsg = rm_msgs::msg::NavigationStatus;
    std::shared_ptr<TerrainSpeed> terrain_speed;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    nav_msgs::msg::Path path;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_suber;
    rclcpp_action::Client<Navigate>::SharedPtr nav_client;
    rclcpp::Subscription<DecisionNavMsg>::SharedPtr decision_suber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_suber;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_suber;
    rclcpp::Publisher<ChassisControlMsg>::SharedPtr navigation_puber;
    rclcpp::Publisher<StatusMsg>::SharedPtr status_puber;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_puber;
    rclcpp::Publisher<BoolMsg>::SharedPtr mode_puber;
    rclcpp::TimerBase::SharedPtr control_timer;
    DecisionNavMsg decision{};
    geometry_msgs::msg::Twist velocity{};
    GoalHandle::SharedPtr goal_handle;
    double command_timeout, odom_timeout;
    double last_command = -1, last_odom = -1, last_velocity = -1;
    bool pending = false, pending_cancel = false, need_goal = false;
    uint64_t active_id = 0, cancel_id = 0;
    uint64_t generation = 0;
    uint8_t state = StatusMsg::IDLE;
    std::string reason = "waiting";
    void DecisionCallback(DecisionNavMsg::SharedPtr msg);
    void send_goal(uint64_t id);
    void cancel_goal();
    void control_cbk();
    void pub_status(uint8_t value, const std::string &text);
};
