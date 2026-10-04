#include "sentry_planner_node.hpp"

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()
    );
    tf_buffer->setCreateTimerInterface(timer_interface);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    odom_suber = create_subscription<OdomMsg>("/odin1/odometry_highfreq", 10, std::bind(&SentryPlannerNode::OdomCallBack, this, std::placeholders::_1));
    chassis_data_suber = create_subscription<ChassisDataMsg>("/chassis_data", 10, std::bind(&SentryPlannerNode::ChassisInfoCallback, this, std::placeholders::_1));
    decision_suber = create_subscription<DecisionNavMsg>(
        "/decision2nav", 10,
        std::bind(&SentryPlannerNode::DecisionCallback, this, std::placeholders::_1));
    cloud_suber = create_subscription<CloudMsg>(
        "/odin1/cloud_raw", rclcpp::SensorDataQoS(),
        std::bind(&SentryPlannerNode::CloudCallback, this, std::placeholders::_1));
    navigation_puber = create_publisher<NavigationMsg>("/navigation2mcu", 10);
    info_log(name, "has been built!");
}

// 底盘信息回调
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    chassis = *msg;
}

// 定位节点位置回调
void SentryPlannerNode::OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom = *msg;
}

void SentryPlannerNode::DecisionCallback(DecisionNavMsg::SharedPtr msg) {
    decision = *msg;
}

void SentryPlannerNode::CloudCallback(CloudMsg::SharedPtr msg) {
    cloud = msg;
}
