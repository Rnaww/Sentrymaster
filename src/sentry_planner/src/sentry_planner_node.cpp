#include "sentry_planner_node.hpp"
#include <stdexcept>
#include <sensor_msgs/point_cloud2_iterator.hpp>

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    const int test_param = declare_parameter<int>("test_param", 1);
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
    control_timer = create_wall_timer(std::chrono::milliseconds(20), std::bind(&SentryPlannerNode::Tick, this));
    RCLCPP_INFO(get_logger(), "PLANNER YAML: test_param=%d", test_param);
    info_log(name, "has been built!");
}

// 底盘信息回调
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    chassis = *msg;
    ++chassis_count;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "PLANNER RX chassis_data: vx=%.3f, vy=%.3f mm/s, wz=%.3f deg/s",
        msg->vx, msg->vy, msg->wz);
}

// 定位节点位置回调
void SentryPlannerNode::OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom = *msg;
    ++odom_count;
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "PLANNER RX odometry: position=(%.3f, %.3f, %.3f), quaternion=(%.3f, %.3f, %.3f, %.3f)",
        p.x, p.y, p.z, q.x, q.y, q.z, q.w);
}

void SentryPlannerNode::DecisionCallback(DecisionNavMsg::SharedPtr msg) {
    decision = *msg;
    ++decision_count;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "PLANNER RX decision2nav: is_cross=%s, aim=(%.3f, %.3f), move_v=%d mm/s",
        msg->is_cross ? "true" : "false", msg->aim_x, msg->aim_y, msg->move_v);
}

void SentryPlannerNode::CloudCallback(CloudMsg::SharedPtr msg) {
    cloud = msg;
    ++cloud_count;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "PLANNER RX cloud_raw: frame=%s, points=%u",
        msg->header.frame_id.c_str(), msg->width * msg->height);
    if (!msg->data.empty()) {
        try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "PLANNER RX cloud_raw first point=(%.3f, %.3f, %.3f)", *x, *y, *z);
        } catch (const std::runtime_error &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Point cloud XYZ fields unavailable: %s", ex.what());
        }
    }
}

void SentryPlannerNode::Tick() {
    // TEST
    NavigationMsg cmd;
    cmd.vx = 1.0f;
    cmd.vy = 1.0f;
    navigation_puber->publish(cmd);
    ++publish_count;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "PLANNER PUB #%llu navigation2mcu(vx=%.3f, vy=%.3f mm/s); "
        "RX counts odom=%llu chassis=%llu decision2nav=%llu cloud=%llu",
        static_cast<unsigned long long>(publish_count), cmd.vx, cmd.vy,
        static_cast<unsigned long long>(odom_count), static_cast<unsigned long long>(chassis_count),
        static_cast<unsigned long long>(decision_count), static_cast<unsigned long long>(cloud_count));
}
