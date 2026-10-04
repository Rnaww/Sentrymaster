#include "transfer.h"

Transfer::Transfer(rclcpp::Node *node, std::shared_ptr<serial::Serial> _serial_port){
    this->node = node;
    serial_port = _serial_port;

    referee_puber = node->create_publisher<RefereeMsg>("referee_data", 10);
    chassis_puber = node->create_publisher<ChassisMsg>("chassis_data", 10);
    decision.sentry_posture = 3;
    navigation_suber = node->create_subscription<NavMsg>("/navigation2mcu", 10,
        std::bind(&Transfer::navigation_callback, this, std::placeholders::_1));
    decision_suber = node->create_subscription<DecisionMsg>("/decision2mcu", 10,
        std::bind(&Transfer::decision_callback, this, std::placeholders::_1));
}


void Transfer::decision_callback(DecisionMsg::SharedPtr msg){
    decision = *msg;
}

void Transfer::navigation_callback(NavMsg::SharedPtr msg){
    ChassisControl control{};
    control.vx = msg->vx;
    control.vy = msg->vy;
    control.wz = decision.wz;
    control.sentry_posture = decision.sentry_posture;

    send_serial_packet(
        trans_struct_uint8<ChassisControl>(chassis_control_id, control, CRCMode::crc16),
        "chassis packet"
    );
}

void Transfer::handle_datas(const std::vector<uint8_t> &datas){
    if(datas[1] == serial_referee_id){
        RefereeData referee;
        referee = trans_uint8_struct<RefereeData>(datas, serial_referee_id, CRCMode::crc16);
        pub_referee_msg(referee);
        // info_log("recv referee info");
    }else if(datas[1] == serial_move_id) {
        ChassisData chassis;
        chassis = trans_uint8_struct<ChassisData>(datas, serial_move_id, CRCMode::crc16);
        pub_chassis_msg(chassis);
        // info_log("recv chassis data");
    }
}

void Transfer::pub_referee_msg(const RefereeData &referee_info){
    RefereeMsg msg;
    msg.game_time = referee_info.game_time;
    msg.sentry_posture = referee_info.sentry_posture;
    referee_puber->publish(msg);
}

void Transfer::pub_chassis_msg(const ChassisData &chassis_info){
    ChassisMsg msg;
    msg.vx = chassis_info.vx;
    msg.vy = chassis_info.vy;
    msg.wz = chassis_info.wz;
    chassis_puber->publish(msg);
}

bool Transfer::send_serial_packet(const std::vector<uint8_t> &datas, const std::string &tag) {
    if (!serial_port) {
        error_log("serial port is null, skip ", tag);
        return false;
    }
    if (!serial_port->isOpen()) {
        error_log("serial port is not open, skip ", tag);
        return false;
    }

    try {
        serial_port->write(datas);
        return true;
    } catch (const std::exception &ex) {
        error_log("failed to send ", tag, ": ", ex.what());
        return false;
    }
}
