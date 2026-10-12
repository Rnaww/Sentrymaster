#pragma once
#include "local_optimization/local_optimizer.hpp"

LocalOptimizer::LocalOptimizer(rclcpp::Node *node){
    maxVelMag = node->declare_parameter<double>("localOptimizer.MaxVelMag", 0.0);
    maxBdrMag = node->declare_parameter<double>("localOptimizer.MaxBdrMag", 0.0);
    maxTiltAngle = node->declare_parameter<double>("localOptimizer.MaxTiltAngle", 0.0);
    minThrust = node->declare_parameter<double>("localOptimizer.MinThrust", 0.0);
    maxThrust = node->declare_parameter<double>("localOptimizer.MaxThrust", 0.0);
    vehicleMass = node->declare_parameter<double>("localOptimizer.VehicleMass", 0.0);
    gravAcc = node->declare_parameter<double>("localOptimizer.GravAcc", 9.81);
    horizDrag = node->declare_parameter<double>("localOptimizer.HorizDrag", 0.0);
    vertDrag = node->declare_parameter<double>("localOptimizer.VertDrag", 0.0);
    parasDrag = node->declare_parameter<double>("localOptimizer.ParasDrag", 0.0);
    speedEps = node->declare_parameter<double>("localOptimizer.SpeedEps", 0.0);
    weightT = node->declare_parameter<double>("localOptimizer.WeightT", 0.0);
    chiVec = node->declare_parameter<std::vector<double>>("localOptimizer.ChiVec", {});
    smoothingEps = node->declare_parameter<double>("localOptimizer.SmoothingEps", 0.0);
    integralIntervs = node->declare_parameter<int>("localOptimizer.IntegralIntervs", 0);
    relCostTol = node->declare_parameter<double>("localOptimizer.RelCostTol", 0.0);
    distanceCostThreshold = node->declare_parameter<double>("localOptimizer.DistanceCostThreshold", 0.0);
    distanceCostWeight = node->declare_parameter<double>("localOptimizer.DistanceCostWeight", 0.0);
    mapResolution = node->get_parameter("globalPlanner.resolution").as_double();
    mapBound = node->get_parameter("MapBound").as_double_array();
}

bool LocalOptimizer::get_trajectory(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::MatrixX4d> &SFCs, double *esdf){
    traj.clear();
    Eigen::Matrix3d iniState;
    Eigen::Matrix3d finState;
    // iniState << path.front(), last_v, Eigen::Vector3d::Zero();
    iniState << path.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    finState << path.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    
    gcopter::GCOPTER_PolytopeSFC gcopter;

    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(7);
    Eigen::VectorXd physicalParams(13);
    magnitudeBounds(0) = maxVelMag;
    magnitudeBounds(1) = maxBdrMag;
    magnitudeBounds(2) = maxTiltAngle;
    magnitudeBounds(3) = minThrust;
    magnitudeBounds(4) = maxThrust;
    penaltyWeights(0) = chiVec[0];
    penaltyWeights(1) = chiVec[1];
    penaltyWeights(2) = chiVec[2];
    penaltyWeights(3) = chiVec[3];
    penaltyWeights(4) = chiVec[4];
    penaltyWeights(5) = distanceCostThreshold;
    penaltyWeights(6) = distanceCostWeight;
    physicalParams(0) = vehicleMass;
    physicalParams(1) = gravAcc;
    physicalParams(2) = horizDrag;
    physicalParams(3) = vertDrag;
    physicalParams(4) = parasDrag;
    physicalParams(5) = speedEps;
    physicalParams(6) = mapResolution;
    physicalParams(7) = mapBound[0];
    physicalParams(8) = mapBound[2];
    physicalParams(9) = mapBound[4];
    physicalParams(10) = static_cast<int>((mapBound[1] - mapBound[0]) / mapResolution);
    physicalParams(11) = static_cast<int>((mapBound[3] - mapBound[2]) / mapResolution);
    physicalParams(12) = static_cast<int>((mapBound[5] - mapBound[4]) / mapResolution);
    const int quadratureRes = integralIntervs;

    if (!gcopter.setup(weightT, iniState, finState, SFCs, 5.0,
                        smoothingEps, quadratureRes, magnitudeBounds, penaltyWeights,
                        physicalParams, esdf)) {
        error_log("setup gcopter fail!");
        traj.clear();
        return false;
    }

    if (std::isinf(gcopter.optimize(traj, relCostTol))) {
        error_log("optimize gcopter fail!");
        traj.clear();
        return false;
    }

    // info_log("---traj config---");
    // info_log("traj piece:", traj.getPieceNum());
    // for (int i = 0; i < traj.getPieceNum(); i ++){
    //     info_log("piece ", i, " time: ", traj.pieces[i].getDuration());
    // }
    return true;
}

bool LocalOptimizer::has_traj(){
    if (traj.getPieceNum()){
        return true;
    }
    return false;
}

void LocalOptimizer::clear_traj(){
    traj.clear();
}

Trajectory<5>* LocalOptimizer::get_traj(){
    return &traj;
}

Eigen::Vector3d LocalOptimizer::get_v_directioin(){
    Eigen::Vector3d p0 = traj.getPos(0.0);
    Eigen::Vector3d dir;
    for (double t = 0.01; t <= traj.getTotalDuration(); t += 0.01) {
        dir = traj.getPos(t) - p0;
        dir.z() = 0.0;
        if (dir.norm() > 0.1) break;  // 前瞻 0.2m
    }
    return dir.normalized() * 1000;
}

double LocalOptimizer::get_curvature(){
    if (!has_traj()) {
        return 0.0;
    }

    const double speed_thresh = 1e-6;
    const double total_duration = traj.getTotalDuration();
    for (double t = 0.0; t <= total_duration; t += 0.01) {
        Eigen::Vector3d vel = traj.getVel(t);
        Eigen::Vector3d acc = traj.getAcc(t);
        vel.z() = 0.0;
        acc.z() = 0.0;

        const double speed = vel.norm();
        if (speed > speed_thresh) {
            return vel.cross(acc).z() / std::pow(speed, 3);
        }
    }

    return 0.0;
}
