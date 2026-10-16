#pragma once
#include "common.h"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"

// 局部轨迹规划器
class LocalOptimizer{
private:
    Trajectory<5> traj;
    int integralIntervs;
    double maxVelMag, maxBdrMag, maxTiltAngle, minThrust, maxThrust;
    double vehicleMass, gravAcc, horizDrag, vertDrag, parasDrag, speedEps;
    double weightT, smoothingEps, relCostTol, distanceCostThreshold, distanceCostWeight;
    double mapResolution;
    std::vector<double> mapBound;
    std::vector<double> chiVec;

public:
    LocalOptimizer(rclcpp::Node *node);

    // 得到轨迹
    bool get_trajectory(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::MatrixX4d> &SFCs,
        double *esdf, const Eigen::Vector3d &start_velocity);
    bool has_traj();
    void clear_traj();

    Trajectory<5>* get_traj();
    // 获取路径方向，取距离起点最近的点并且转换为mm/s
    Eigen::Vector3d get_v_directioin();
    // 获取当前点的平面曲率
    double get_curvature();
};
