#include "common.h"

struct Config{
    int rate;
    double reach_thresh;
    double end_dis;
    double slow_down_pow;
    std::vector<double> mapBound;
    double rotate_feedforward;
    double back_distance;

    // 障碍物点云缓冲帧数
    int obs_cloud_buffer_frame;

    explicit Config(rclcpp::Node* nh){
        rate = nh->declare_parameter<int>("rate", 50);
        reach_thresh = nh->declare_parameter<double>("reach_thresh", 0.5);
        end_dis = nh->declare_parameter<double>("end_dis", 1.0);
        slow_down_pow = nh->declare_parameter<double>("slow_down_pow", 1.0);
        mapBound = nh->declare_parameter<std::vector<double>>("MapBound", {});
        obs_cloud_buffer_frame = nh->declare_parameter<int>("obs_cloud_buffer_frame", 0);
        rotate_feedforward = nh->declare_parameter<double>("rotate_feedforward", 0.0);
        back_distance = nh->declare_parameter<double>("back_distance", 1.0);
    }
};