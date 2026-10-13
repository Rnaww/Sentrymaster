#include "sentry_planner_node.hpp"
#include "terrain_speed.hpp"

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    terrain_speed = std::make_shared<TerrainSpeed>(this);
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    path_suber = create_subscription<nav_msgs::msg::Path>("/planned_path", 10, [this](nav_msgs::msg::Path::SharedPtr msg){ path = *msg; });
    command_timeout = declare_parameter("command_timeout", 0.5);
    odom_timeout = declare_parameter("odom_timeout", 0.5);
    nav_client = rclcpp_action::create_client<Navigate>(this, "/navigate_to_pose");
    decision_suber = create_subscription<DecisionNavMsg>("/decision2nav", 10, std::bind(&SentryPlannerNode::DecisionCallback, this, std::placeholders::_1));
    velocity_suber = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){ velocity = *msg; velocity_yaw = yaw; last_velocity = now().seconds(); });
    odom_suber = create_subscription<OdomMsg>("/navigation_odom", 10, [this](OdomMsg::SharedPtr msg){
        try{
            auto trans = tf_buffer->lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
            auto &p = msg->pose.pose.position;
            position = tf2::transformToEigen(trans.transform)*Eigen::Vector3d(p.x,p.y,p.z);
            auto &q = msg->pose.pose.orientation;
            Eigen::Matrix3d rotation = tf2::transformToEigen(trans.transform).linear()*Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
            yaw = std::atan2(rotation(1,0),rotation(0,0));
            yaw_rate = msg->twist.twist.angular.z;
            odom_stamp = rclcpp::Time(msg->header.stamp).seconds();
            last_odom = now().seconds();
        }catch(const tf2::TransformException &ex){}
    });
    navigation_puber = create_publisher<ChassisControlMsg>("/navigation2mcu", 10);
    speed_puber = create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);
    mode_puber = create_publisher<BoolMsg>("/navigation_cross_mode", rclcpp::QoS(1).transient_local());
    control_timer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SentryPlannerNode::control_cbk, this));
    info_log("NAV2 BRIDGE YAML: command_timeout=", command_timeout, " odom_timeout=", odom_timeout);
}

void SentryPlannerNode::log_state(uint8_t value, const std::string &text){
    if (state != value || reason != text) info_log("NAV2 state=", static_cast<int>(value), text);
    state = value;
    reason = text;
}

void SentryPlannerNode::cancel_goal(){
    generation++;
    need_goal = false;
    retry_time = 0;
    velocity = geometry_msgs::msg::Twist();
    path.poses.clear();
    last_velocity = -1;
    if (goal_handle){
        pending_cancel = true;
        cancel_id = active_id;
        nav_client->async_cancel_goal(goal_handle);
        goal_handle.reset();
    }
}

void SentryPlannerNode::DecisionCallback(DecisionNavMsg::SharedPtr msg){
    if (!std::isfinite(msg->aim_x) || !std::isfinite(msg->aim_y) || msg->move_v < 0){
        cancel_goal();
        decision.move_v = 0;
        log_state(FAILED, "invalid target");
        return;
    }
    bool changed = msg->aim_x != decision.aim_x || msg->aim_y != decision.aim_y || msg->is_cross != decision.is_cross;
    bool resume = msg->move_v > 0 && (decision.move_v == 0 || last_command < 0 || now().seconds()-last_command >= command_timeout);
    if (changed || resume || (decision.move_v > 0 && msg->move_v == 0)){
        cancel_goal();
        need_goal = msg->move_v > 0;
        log_state(msg->move_v > 0 ? IDLE : CANCELED, "new command");
    }
    decision = *msg;
    last_command = now().seconds();
    nav2_msgs::msg::SpeedLimit limit;
    limit.header.stamp = now();
    limit.speed_limit = std::min(4.5, decision.move_v/1000.0);
    limit.percentage = false;
    speed_puber->publish(limit);
    BoolMsg mode;
    mode.data = decision.is_cross;
    mode_puber->publish(mode);
}

void SentryPlannerNode::send_goal(uint64_t id){
    Navigate::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = decision.aim_x;
    goal.pose.pose.position.y = decision.aim_y;
    goal.pose.pose.orientation.w = 1;
    pending = true;
    need_goal = false;
    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback = [this, id](GoalHandle::SharedPtr handle){
        pending = false;
        if (id != generation){
            if (handle){
                pending_cancel = true;
                cancel_id = id;
                nav_client->async_cancel_goal(handle);
            }
            return;
        }
        goal_handle = handle;
        active_id = id;
        if (handle) log_state(RUNNING, "following PB PID");
        else{ log_state(FAILED, "goal rejected"); need_goal = true; retry_time = now().seconds()+2; }
    };
    options.result_callback = [this, id](const GoalHandle::WrappedResult &result){
        if (pending_cancel && cancel_id == id) pending_cancel = false;
        if (id != generation) return;
        goal_handle.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) log_state(SUCCEEDED, "reached");
        else if (result.code == rclcpp_action::ResultCode::CANCELED) log_state(CANCELED, "canceled");
        else{ log_state(FAILED, "Nav2 aborted, retrying"); need_goal = true; retry_time = now().seconds()+2; }
        velocity = geometry_msgs::msg::Twist();
    };
    nav_client->async_send_goal(goal, options);
}

void SentryPlannerNode::control_cbk(){
    bool command_valid = last_command >= 0 && now().seconds()-last_command < command_timeout && decision.move_v > 0;
    bool odom_valid = last_odom >= 0 && now().seconds()-last_odom < odom_timeout;
    if (command_valid && odom_valid && !terrain_speed->navigationFree(position)){
        if (!recovering){ cancel_goal(); recovering = true; }
        Eigen::Vector3d map_v = terrain_speed->recoveryVelocity(position)*std::min(1000.0,decision.move_v/0.15);
        double heading = yaw+yaw_rate*std::clamp(now().seconds()-odom_stamp,0.0,0.05);
        ChassisControlMsg output{};
        output.vx = std::cos(heading)*map_v.x()+std::sin(heading)*map_v.y();
        output.vy = -std::sin(heading)*map_v.x()+std::cos(heading)*map_v.y();
        navigation_puber->publish(output);
        log_state(map_v.norm() > 0 ? RUNNING : FAILED,"restore navigation clearance");
        return;
    }
    if (recovering){
        recovering = false;
        need_goal = command_valid && odom_valid;
        log_state(need_goal ? IDLE : CANCELED,"clearance restored");
    }
    if ((!command_valid || !odom_valid) && (goal_handle || need_goal)){
        cancel_goal();
        log_state(command_valid ? FAILED : CANCELED, "input unavailable");
    }
    if (command_valid && odom_valid && need_goal && now().seconds() >= retry_time && !pending && !pending_cancel && nav_client->action_server_is_ready()) send_goal(generation);
    double speed_limit = terrain_speed->getLimit(position, path, decision.move_v/1000.0);
    nav2_msgs::msg::SpeedLimit limit_msg;
    limit_msg.header.stamp = now();
    limit_msg.speed_limit = speed_limit;
    limit_msg.percentage = false;
    speed_puber->publish(limit_msg);
    ChassisControlMsg output{};
    if (command_valid && odom_valid && state == RUNNING && last_velocity >= 0 && now().seconds()-last_velocity < 0.2){
        // 按速度产生时的 yaw 还原 map 方向，再补偿自旋期间的定位延迟。
        double speed = std::hypot(velocity.linear.x, velocity.linear.y);
        double limit = speed_limit;
        double scale = speed > limit ? limit/speed : 1;
        if (std::isfinite(speed)){
            Eigen::Vector3d map_v(std::cos(velocity_yaw)*velocity.linear.x-std::sin(velocity_yaw)*velocity.linear.y,
                                  std::sin(velocity_yaw)*velocity.linear.x+std::cos(velocity_yaw)*velocity.linear.y,0);
            map_v = terrain_speed->checkedVelocity(position,map_v*scale);
            double heading = yaw+yaw_rate*std::clamp(now().seconds()-odom_stamp,0.0,0.05);
            output.vx = (std::cos(heading)*map_v.x()+std::sin(heading)*map_v.y())*1000;
            output.vy = (-std::sin(heading)*map_v.x()+std::cos(heading)*map_v.y())*1000;
        }
    }
    navigation_puber->publish(output);

}
