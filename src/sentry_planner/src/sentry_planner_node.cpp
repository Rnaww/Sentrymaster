#include "sentry_planner_node.hpp"
#include "planning_core.hpp"

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    planning_core = std::make_shared<PlanningCore>(this);
    odom_timeout = declare_parameter("odom_timeout", 0.5);
    command_timeout = declare_parameter("command_timeout", 0.5);
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_buffer->setCreateTimerInterface(timer_interface);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_puber = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_suber = create_subscription<OdomMsg>("/odin1/odometry_highfreq", 10, std::bind(&SentryPlannerNode::OdomCallBack, this, std::placeholders::_1));
    chassis_data_suber = create_subscription<ChassisDataMsg>("/chassis_data", 10, std::bind(&SentryPlannerNode::ChassisInfoCallback, this, std::placeholders::_1));
    decision_suber = create_subscription<DecisionNavMsg>("/decision2nav", 10, std::bind(&SentryPlannerNode::DecisionCallback, this, std::placeholders::_1));
    cloud_suber = create_subscription<PcdMsg>("/odin1/cloud_raw", rclcpp::SensorDataQoS(), std::bind(&SentryPlannerNode::CloudCallback, this, std::placeholders::_1));
    navigation_puber = create_publisher<ChassisControlMsg>("/navigation2mcu", 10);
    status_puber = create_publisher<StatusMsg>("/navigation_status", 10);
    path_puber = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    control_timer = create_wall_timer(std::chrono::milliseconds(1000 / planning_core->config->rate), std::bind(&SentryPlannerNode::control_cbk, this));
    info_log(name, "has been built!");
}

void SentryPlannerNode::OdomCallBack(OdomMsg::SharedPtr msg){
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
        current_odom_pos_2d = world_vehicle.translation();
        current_odom_pos_2d.z() = 0.05;
        yaw = std::atan2(world_vehicle.linear()(1,0), world_vehicle.linear()(0,0));
    }catch(const tf2::TransformException &ex){
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
        pub_status(StatusMsg::FAILED, "invalid target");
        return;
    }
    if (msg->aim_x != decision.aim_x || msg->aim_y != decision.aim_y || msg->is_cross != decision.is_cross) last_state = 255;
    decision = *msg;
    last_command_time = now().seconds();
}

void SentryPlannerNode::CloudCallback(PcdMsg::SharedPtr msg){
    cloud = msg;
}

void SentryPlannerNode::pub_status(uint8_t state, const std::string &reason){
    StatusMsg msg;
    msg.aim_x = decision.aim_x;
    msg.aim_y = decision.aim_y;
    msg.state = state;
    msg.reason = reason;
    status_puber->publish(msg);
    if (state != last_state){
        info_log("NAV state=", static_cast<int>(state), " goal=", msg.aim_x, msg.aim_y, reason);
        last_state = state;
    }
}

void SentryPlannerNode::control_cbk(){
    ChassisControlMsg chassis_cmd{};
    if (last_command_time < 0 || now().seconds()-last_command_time > command_timeout || decision.move_v == 0){
        pub_status(StatusMsg::CANCELED, "stopped");
    }else if (last_odom_time < 0 || now().seconds()-last_odom_time > odom_timeout){
        pub_status(StatusMsg::FAILED, "odometry/TF unavailable");
    }else{
        Eigen::Vector3d goal(decision.aim_x, decision.aim_y, 0.05);
        double now_dis = (current_odom_pos_2d-goal).head<2>().norm();
        if (now_dis <= planning_core->config->reach_thresh){
            pub_status(StatusMsg::SUCCEEDED, "reached");
        }else if (planning_core->plan(current_odom_pos_2d, goal, decision.is_cross)){
            // 沿用旧前瞻方向和末端减速。MINCO 时标不作为执行时标。
            Eigen::Vector3d map_v = planning_core->local_optimizer->get_v_directioin();
            if (map_v.allFinite() && map_v.head<2>().norm() > 1e-6){
                double new_spd = std::min(3500.0, static_cast<double>(decision.move_v));
                if (now_dis < planning_core->config->end_dis) new_spd *= std::pow(now_dis/planning_core->config->end_dis, planning_core->config->slow_down_pow);
                map_v = map_v.normalized()*new_spd;
                // map -> vehicle，右手系，yaw 正方向逆时针。
                chassis_cmd.vx = std::cos(yaw)*map_v.x() + std::sin(yaw)*map_v.y();
                chassis_cmd.vy = -std::sin(yaw)*map_v.x() + std::cos(yaw)*map_v.y();
                pub_status(StatusMsg::RUNNING, "following MINCO");
                path_puber->publish(planning_core->getPath(now()));
            }else pub_status(StatusMsg::FAILED, "invalid direction");
        }else pub_status(StatusMsg::FAILED, planning_core->last_error);
    }
    // 包含停车时的零平移消息，UART 可继续合并决策的 wz。
    navigation_puber->publish(chassis_cmd);
}
