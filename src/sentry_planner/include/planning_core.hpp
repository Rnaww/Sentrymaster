#pragma once
#include "config.hpp"
#include "global_planner/global_planner.hpp"
#include "local_optimization/sfc_generator.hpp"
#include "local_optimization/local_optimizer.hpp"
#include <nav_msgs/msg/path.hpp>

// 旧框架的搜索、安全走廊和轨迹优化。
class PlanningCore{
public:
    std::string last_error;
    std::shared_ptr<Config> config;
    std::shared_ptr<GlobalPathPlanner> global_path_planner;
    std::shared_ptr<SFCgenerator> sfc_generator;
    std::shared_ptr<LocalOptimizer> local_optimizer;

    PlanningCore(rclcpp::Node *node){
        config = std::make_shared<Config>(node);
        global_path_planner = std::make_shared<GlobalPathPlanner>(node);
        sfc_generator = std::make_shared<SFCgenerator>(node);
        local_optimizer = std::make_shared<LocalOptimizer>(node);
        info_log("701 YAML: resolution=", global_path_planner->resolution_,
                 " rate=", config->rate, " reach_thresh=", config->reach_thresh);
    }

    bool isFree(const Eigen::Vector3d &point){
        return global_path_planner->coorIsInMap2d(point) &&
               global_path_planner->isFree(global_path_planner->coord2gridIndex(point));
    }

    bool segmentFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
        int n = std::max(1, static_cast<int>(std::ceil((b-a).head<2>().norm() / 0.05)));
        for (int i = 0; i <= n; i++){
            if (!isFree(a + (b-a) * (static_cast<double>(i)/n))) return false;
        }
        return true;
    }

    bool plan(Eigen::Vector3d start, Eigen::Vector3d goal, bool is_cross){
        local_optimizer->clear_traj();
        start.z() = 0.05;
        goal.z() = 0.05;
        if (is_cross) global_path_planner->setObsCross();
        else global_path_planner->resetObs();
        if (!isFree(start) || !isFree(goal)){ last_error = "occupied start/goal"; return false; }
        std::vector<Eigen::Vector3d> route;
        if (!global_path_planner->GetPath(route, start, goal)){ last_error = "A* failed"; return false; }
        if ((route.back()-goal).head<2>().norm() > 0.2){ last_error = "partial A* path"; return false; }
        route.front() = start;
        route.back() = goal;
        for (auto &point : route) point.z() = 0.05;
        std::vector<Eigen::Vector3d> pc, nearby_pc;
        global_path_planner->getOccupyVector(pc);
        Eigen::Vector3d low = start, high = start;
        for (auto &point : route){
            low = low.cwiseMin(point);
            high = high.cwiseMax(point);
        }
        for (auto &point : pc){
            if (point.x() > low.x()-2 && point.x() < high.x()+2 &&
                point.y() > low.y()-2 && point.y() < high.y()+2){
                // 单层障碍点位于 z 边界会被旧 SFC 严格不等式过滤；在层内表示障碍柱。
                double half_cell = global_path_planner->resolution_*0.5 + 0.01;
                for (double dx : {-half_cell, half_cell}){
                    for (double dy : {-half_cell, half_cell}){
                        nearby_pc.emplace_back(point.x()+dx, point.y()+dy, 0.025);
                        nearby_pc.emplace_back(point.x()+dx, point.y()+dy, 0.075);
                    }
                }
            }
        }
        global_path_planner->updateESDF(start, nearby_pc);
        std::vector<Eigen::MatrixX4d> SFCs;
        sfc_generator->getSFC(route, nearby_pc, SFCs);
        if (SFCs.empty()){ last_error = "empty SFC"; return false; }
        if (!local_optimizer->get_trajectory(route, SFCs, global_path_planner->esdf)){ last_error = "MINCO failed"; return false; }
        // 优化成功后检查整条曲线，失败时不执行未经检查的路径。
        auto traj = local_optimizer->get_traj();
        Eigen::Vector3d last = start;
        int n = std::max(2, static_cast<int>(std::ceil(traj->getTotalDuration()/0.02)));
        n = std::min(n, 20000);
        for (int i = 1; i <= n; i++){
            Eigen::Vector3d point = traj->getPos(traj->getTotalDuration()*i/n);
            if (!point.allFinite() || !segmentFree(last, point)){
                last_error = "curve outside free map at " + std::to_string(point.x()) + "," + std::to_string(point.y());
                local_optimizer->clear_traj();
                return false;
            }
            last = point;
        }
        return true;
    }

    nav_msgs::msg::Path getPath(const rclcpp::Time &stamp){
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = stamp;
        if (!local_optimizer->has_traj()) return path;
        auto traj = local_optimizer->get_traj();
        int n = std::max(2, static_cast<int>(std::ceil(traj->getTotalDuration()/0.01)));
        n = std::min(n, 20000);
        for (int i = 0; i <= n; i++){
            Eigen::Vector3d point = traj->getPos(traj->getTotalDuration()*i/n);
            if (!path.poses.empty()){
                auto &last = path.poses.back().pose.position;
                if (i < n && std::hypot(point.x()-last.x, point.y()-last.y) < 0.025) continue;
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }
};
