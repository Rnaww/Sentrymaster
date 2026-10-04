#pragma once
#include "common.h"
#include "serial/serial.h"
#include "struct_def.hpp"
#include "rm_msgs/msg/decision_mcu_cmd.hpp"
#include "rm_msgs/msg/navigation_mcu_cmd.hpp"

class Transfer{
private:
    using NavMsg = rm_msgs::msg::NavigationMcuCmd;
    using DecisionMsg = rm_msgs::msg::DecisionMcuCmd;
    rclcpp::Node *node = nullptr;
    rclcpp::Publisher<RefereeMsg>::SharedPtr referee_puber;
    rclcpp::Publisher<ChassisMsg>::SharedPtr chassis_puber;

    rclcpp::Subscription<NavMsg>::SharedPtr navigation_suber;
    rclcpp::Subscription<DecisionMsg>::SharedPtr decision_suber;
    DecisionMsg decision{};
    void navigation_callback(NavMsg::SharedPtr msg);
    void decision_callback(DecisionMsg::SharedPtr msg);

    std::shared_ptr<serial::Serial> serial_port;

    bool send_serial_packet(const std::vector<uint8_t> &datas, const std::string &tag);

public:
    Transfer(rclcpp::Node *node, std::shared_ptr<serial::Serial> _serial_port);

    void handle_datas(const std::vector<uint8_t> &datas);
    void pub_referee_msg(const RefereeData &referee_info);
    void pub_chassis_msg(const ChassisData &chassis_info);
};
