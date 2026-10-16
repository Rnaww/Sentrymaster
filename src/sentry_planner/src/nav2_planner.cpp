#include "planning_core.hpp"
#include "terrain_speed.hpp"
#include <atomic>
#include <mutex>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>

// 用 Nav2 插件调用相同的旧规划算法。
class SentryMincoPlanner : public nav2_core::GlobalPlanner{
public:
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override{
        node = parent.lock();
        std::vector<rclcpp::Parameter> parameters;
        std::string prefix = name + ".";
        for (auto &entry : node->get_node_parameters_interface()->get_parameter_overrides()){
            auto &key = entry.first;
            if (key.find(prefix) != 0 || key == prefix + "plugin")
                continue;
            if (!node->has_parameter(key))
                node->declare_parameter(key, entry.second);
            parameters.emplace_back(key.substr(prefix.size()), entry.second);
        }
        core_node = std::make_shared<rclcpp::Node>(
            "sentry_planning_core", rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(parameters));
        core = std::make_shared<PlanningCore>(core_node.get());
        terrain_speed = std::make_shared<TerrainSpeed>(core_node.get());
        odom_timeout = core_node->declare_parameter("odom_timeout", 0.5);
        tf_buffer = tf;
        odom_suber = node->create_subscription<OdomMsg>(
            "/navigation_odom", 10, [this](OdomMsg::SharedPtr msg){
                std::lock_guard<std::mutex> lock(odom_mutex);
                odom = *msg;
            });
        mode_suber = node->create_subscription<BoolMsg>(
            "/navigation_cross_mode", rclcpp::QoS(1).transient_local(), [this](BoolMsg::SharedPtr msg){
                is_cross = msg->data;
            });
        path_puber = node->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    }

    void cleanup() override{
        mode_suber.reset();
        odom_suber.reset();
        path_puber.reset();
        terrain_speed.reset();
        core.reset();
        core_node.reset();
    }
    void activate() override{
        path_puber->on_activate();
    }
    void deactivate() override{
        path_puber->on_deactivate();
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) override{
        Eigen::Vector3d a(start.pose.position.x, start.pose.position.y, 0.05);
        Eigen::Vector3d b(goal.pose.position.x, goal.pose.position.y, 0.05);
        OdomMsg current_odom;
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            current_odom = odom;
        }
        Eigen::Vector3d start_velocity = Eigen::Vector3d::Zero();
        double age = node->now().seconds() - rclcpp::Time(current_odom.header.stamp).seconds();
        // 任意起终点的静态规划仍可使用零边界；实测速度只用于机器人当前位置。
        if (!current_odom.header.frame_id.empty() && std::abs(age) <= odom_timeout){
            auto &q = current_odom.pose.pose.orientation;
            auto &v = current_odom.twist.twist.linear;
            auto &p = current_odom.pose.pose.position;
            Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
            Eigen::Vector3d velocity(v.x, v.y, v.z), position(p.x, p.y, p.z);
            if (rotation.coeffs().allFinite() && rotation.norm() > 1e-6 &&
                velocity.allFinite() && position.allFinite()){
                // navigation_odom 的速度在 vehicle 系，先用完整姿态转到地图系，单位 m/s。
                velocity = rotation.normalized() * velocity;
                try{
                    if (current_odom.header.frame_id != "map"){
                        auto trans = tf_buffer->lookupTransform("map", current_odom.header.frame_id, tf2::TimePointZero);
                        auto map_odom = tf2::transformToEigen(trans.transform);
                        velocity = map_odom.linear() * velocity;
                        position = map_odom * position;
                    }
                    velocity.z() = 0;
                    // 允许位姿采样延迟及三个栅格的定位差异，不把当前速度用于远处的虚拟起点。
                    double tolerance = 3 * core->global_path_planner->resolution_ +
                        velocity.head<2>().norm() * std::max(0.0, age);
                    if (velocity.allFinite() && (a - position).head<2>().norm() <= tolerance)
                        start_velocity = velocity;
                } catch (const tf2::TransformException &ex){
                    // 没有可用地图变换时，保持静止边界完成几何规划。
                }
            }
        }
        if (!core->plan(a, b, is_cross, start_velocity))
            throw nav2_core::PlannerException(core->last_error);
        auto path = core->getPath(node->now());
        double length;
        double seconds = terrain_speed->estimateTime(path, length);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
            "PATH: length=%.2f m, estimated travel=%.2f s (rest to rest, excluding task holds)", length, seconds);
        path_puber->publish(path);
        return path;
    }

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Node::SharedPtr core_node;
    std::shared_ptr<PlanningCore> core;
    std::shared_ptr<TerrainSpeed> terrain_speed;
    std::atomic<bool> is_cross{false};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_suber;
    OdomMsg odom;
    std::mutex odom_mutex;
    double odom_timeout;
    rclcpp::Subscription<BoolMsg>::SharedPtr mode_suber;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_puber;
};

PLUGINLIB_EXPORT_CLASS(SentryMincoPlanner, nav2_core::GlobalPlanner)
