#include "common.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rm_msgs/msg/decision_nav_cmd.hpp"
#include "rm_msgs/msg/navigation_mcu_cmd.hpp"

class SentryPlannerNode : public rclcpp::Node {
public:
    SentryPlannerNode(std::string name);

private:
    using DecisionNavMsg = rm_msgs::msg::DecisionNavCmd;
    using NavigationMsg = rm_msgs::msg::NavigationMcuCmd;
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_suber;

    rclcpp::Subscription<ChassisDataMsg>::SharedPtr chassis_data_suber;

    rclcpp::Subscription<DecisionNavMsg>::SharedPtr decision_suber;
    rclcpp::Subscription<CloudMsg>::SharedPtr cloud_suber;
    rclcpp::Publisher<NavigationMsg>::SharedPtr navigation_puber;
    // 坐标变换接收者
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    OdomMsg odom{};
    ChassisDataMsg chassis{};
    DecisionNavMsg decision{};
    CloudMsg::SharedPtr cloud;
    void ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg);
    void OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void DecisionCallback(DecisionNavMsg::SharedPtr msg);
    void CloudCallback(CloudMsg::SharedPtr msg);
};
