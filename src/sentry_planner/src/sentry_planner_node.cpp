#include "sentry_planner_node.hpp"
#include "planning_core.hpp"
#include "terrain_speed.hpp"

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    planning_core = std::make_shared<PlanningCore>(this);
    terrain_speed = std::make_shared<TerrainSpeed>(this);
    odom_timeout = declare_parameter("odom_timeout", 0.5);
    command_timeout = declare_parameter("command_timeout", 0.5);
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_buffer->setCreateTimerInterface(timer_interface);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_puber = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_suber = create_subscription<OdomMsg>(
        "/odin1/odometry_highfreq", 10, std::bind(&SentryPlannerNode::OdomCallBack, this, std::placeholders::_1));
    chassis_data_suber = create_subscription<ChassisDataMsg>(
        "/chassis_data", 10, std::bind(&SentryPlannerNode::ChassisInfoCallback, this, std::placeholders::_1));
    decision_suber = create_subscription<DecisionNavMsg>(
        "/decision2nav", 10, std::bind(&SentryPlannerNode::DecisionCallback, this, std::placeholders::_1));
    cloud_suber = create_subscription<PcdMsg>("/odin1/cloud_raw", rclcpp::SensorDataQoS(),
        std::bind(&SentryPlannerNode::CloudCallback, this, std::placeholders::_1));
    navigation_puber = create_publisher<ChassisControlMsg>("/navigation2mcu", 10);
    path_puber = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    control_timer = create_wall_timer(std::chrono::milliseconds(1000 / planning_core->config->rate),
        std::bind(&SentryPlannerNode::control_cbk, this));
    info_log(name, "has been built!");
}

void SentryPlannerNode::OdomCallBack(OdomMsg::SharedPtr msg){
    double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (odom_stamp >= 0 && stamp <= odom_stamp)
        return;
    odom = *msg;
    last_odom_time = now().seconds();
    // Odin 的安装坐标通过原 URDF 转到 vehicle，不把 Odin yaw 当作底盘 yaw。
    try{
        auto fixed = tf_buffer->lookupTransform(msg->child_frame_id, "vehicle", tf2::TimePointZero);
        Eigen::Affine3d world_odin = Eigen::Affine3d::Identity();
        auto &q = msg->pose.pose.orientation;
        world_odin.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        auto &p = msg->pose.pose.position;
        world_odin.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        Eigen::Affine3d world_vehicle = world_odin * tf2::transformToEigen(fixed.transform);
        if (msg->header.frame_id != "map"){
            auto map_world = tf_buffer->lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
            world_vehicle = tf2::transformToEigen(map_world.transform) * world_vehicle;
        }
        double new_yaw = std::atan2(world_vehicle.linear()(1, 0), world_vehicle.linear()(0, 0));
        if (odom_stamp >= 0 && stamp > odom_stamp && stamp - odom_stamp < 0.5){
            odom_velocity = (world_vehicle.translation() - current_odom_pos_2d) / (stamp - odom_stamp);
            odom_velocity.z() = 0;
            yaw_rate = std::atan2(std::sin(new_yaw - yaw), std::cos(new_yaw - yaw)) / (stamp - odom_stamp);
        } else
            odom_velocity.setZero();
        current_odom_pos_2d = world_vehicle.translation();
        current_odom_pos_2d.z() = 0.05;
        yaw = new_yaw;
        odom_stamp = stamp;
    } catch (const tf2::TransformException &ex){
        last_odom_time = -1;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not ready: %s", ex.what());
    }
}

void SentryPlannerNode::ChassisInfoCallback(ChassisDataMsg::SharedPtr msg){
    chassis = *msg;
}

void SentryPlannerNode::DecisionCallback(DecisionNavMsg::SharedPtr msg){
    if (!std::isfinite(msg->aim_x) || !std::isfinite(msg->aim_y) || msg->move_v < 0){
        decision.move_v = 0;
        log_state("invalid target");
        return;
    }
    if (msg->aim_x != decision.aim_x || msg->aim_y != decision.aim_y || msg->is_cross != decision.is_cross)
        last_reason.clear();
    decision = *msg;
    last_command_time = now().seconds();
}

void SentryPlannerNode::CloudCallback(PcdMsg::SharedPtr msg){
    cloud = msg;
}

void SentryPlannerNode::log_state(const std::string &reason){
    if (reason != last_reason){
        info_log("NAV goal=", decision.aim_x, decision.aim_y, reason);
        last_reason = reason;
    }
}

void SentryPlannerNode::control_cbk(){
    ChassisControlMsg chassis_cmd{};
    Eigen::Vector3d map_velocity = Eigen::Vector3d::Zero();
    bool connecting = false;
    Eigen::Vector3d position = current_odom_pos_2d +
        odom_velocity * std::clamp(now().seconds() - odom_stamp, 0.0, odom_timeout);
    if (last_command_time < 0 || now().seconds() - last_command_time > command_timeout || decision.move_v == 0){
        log_state("stopped");
    } else if (last_odom_time < 0 || now().seconds() - last_odom_time > odom_timeout){
        log_state("odometry/TF unavailable");
    } else{
        Eigen::Vector3d goal(decision.aim_x, decision.aim_y, 0.05);
        double now_dis = (position - goal).head<2>().norm();
        if (now_dis <= planning_core->config->reach_thresh){
            log_state("reached");
        } else if (planning_core->plan(position, goal, decision.is_cross, terrain_speed.get())){
            connecting = planning_core->connecting;
            // 沿用旧前瞻方向。MINCO 时标不作为执行时标。
            double lookahead = std::clamp(odom_velocity.head<2>().norm() * 0.2, 0.2, 0.4);
            Eigen::Vector3d map_v = planning_core->getDirection(lookahead);
            if (map_v.allFinite() && map_v.head<2>().norm() > 1e-6){
                auto path = planning_core->getPath(now());
                position = current_odom_pos_2d +
                    odom_velocity * std::clamp(now().seconds() - odom_stamp, 0.0, odom_timeout);
                double new_spd = terrain_speed->getLimit(position, path, decision.move_v / 1000.0) * 1000;
                // terrain_speed 已按制动距离限制终点速度，不再重复乘末端减速系数。
                map_v = map_v.normalized() * new_spd;
                map_v = terrain_speed->checkedVelocity(position, map_v / 1000, connecting) * 1000;
                map_velocity = map_v / 1000.0;
                log_state(connecting                            ? "joining free route"
                        : planning_core->fallback_route.empty() ? "following MINCO"
                                                                : "following A*");
                path_puber->publish(path);
            } else
                log_state("invalid direction");
        } else
            log_state(planning_core->last_error);
    }
    bool emergency = last_command_time < 0 || now().seconds() - last_command_time > command_timeout ||
        last_odom_time < 0 || now().seconds() - last_odom_time > odom_timeout;
    map_velocity = terrain_speed->limitVelocity(map_velocity, now().seconds(), emergency);
    map_velocity = terrain_speed->checkedVelocity(position, map_velocity, connecting);
    terrain_speed->acceptVelocity(map_velocity);
    double heading = yaw + yaw_rate * std::clamp(now().seconds() - odom_stamp, 0.0, odom_timeout);
    chassis_cmd.vx = (std::cos(heading) * map_velocity.x() + std::sin(heading) * map_velocity.y()) * 1000;
    chassis_cmd.vy = (-std::sin(heading) * map_velocity.x() + std::cos(heading) * map_velocity.y()) * 1000;
    navigation_puber->publish(chassis_cmd);
}
