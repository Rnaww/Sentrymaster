#include "planning_core.hpp"
#include <atomic>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>

// 用 Nav2 插件调用相同的旧规划算法。
class SentryMincoPlanner : public nav2_core::GlobalPlanner{
public:
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override{
        node = parent.lock();
        std::vector<rclcpp::Parameter> parameters;
        std::string prefix = name+".";
        for (auto &entry : node->get_node_parameters_interface()->get_parameter_overrides()){
            auto &key = entry.first;
            if (key.find(prefix) != 0 || key == prefix+"plugin") continue;
            if (!node->has_parameter(key)) node->declare_parameter(key, entry.second);
            parameters.emplace_back(key.substr(prefix.size()), entry.second);
        }
        core_node = std::make_shared<rclcpp::Node>("sentry_planning_core", rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(parameters));
        core = std::make_shared<PlanningCore>(core_node.get());
        mode_suber = node->create_subscription<BoolMsg>("/navigation_cross_mode", rclcpp::QoS(1).transient_local(), [this](BoolMsg::SharedPtr msg){ is_cross = msg->data; });
        path_puber = node->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    }

    void cleanup() override{
        mode_suber.reset();
        path_puber.reset();
        core.reset();
        core_node.reset();
    }
    void activate() override{ path_puber->on_activate(); }
    void deactivate() override{ path_puber->on_deactivate(); }

    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                  const geometry_msgs::msg::PoseStamped &goal) override{
        Eigen::Vector3d a(start.pose.position.x, start.pose.position.y, 0.05);
        Eigen::Vector3d b(goal.pose.position.x, goal.pose.position.y, 0.05);
        if (!core->plan(a, b, is_cross)) throw nav2_core::PlannerException(core->last_error);
        auto path = core->getPath(node->now());
        path_puber->publish(path);
        return path;
    }
private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Node::SharedPtr core_node;
    std::shared_ptr<PlanningCore> core;
    std::atomic<bool> is_cross{false};
    rclcpp::Subscription<BoolMsg>::SharedPtr mode_suber;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_puber;
};

PLUGINLIB_EXPORT_CLASS(SentryMincoPlanner, nav2_core::GlobalPlanner)
