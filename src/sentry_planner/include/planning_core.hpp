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
    rclcpp::Node *node;
    std::vector<Eigen::Vector3d> fallback_route;
    std::vector<Eigen::Vector3d> search_route;
    std::shared_ptr<Config> config;
    std::shared_ptr<GlobalPathPlanner> global_path_planner;
    std::shared_ptr<SFCgenerator> sfc_generator;
    std::shared_ptr<LocalOptimizer> local_optimizer;

    PlanningCore(rclcpp::Node *node):node(node){
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
        if (!isFree(a) || !isFree(b)) return false;
        auto index = global_path_planner->coord2gridIndex(a);
        auto end = global_path_planner->coord2gridIndex(b);
        double resolution = global_path_planner->resolution_;
        Eigen::Vector3d delta = b-a;
        int sx = delta.x() > 0 ? 1 : -1, sy = delta.y() > 0 ? 1 : -1;
        double tx = INFINITY, ty = INFINITY;
        double dx = INFINITY, dy = INFINITY;
        if (index.x() != end.x()){
            double boundary = global_path_planner->gl_low_.x()+(index.x()+(sx > 0))*resolution;
            tx = (boundary-a.x())/delta.x();
            dx = resolution/std::abs(delta.x());
        }
        if (index.y() != end.y()){
            double boundary = global_path_planner->gl_low_.y()+(index.y()+(sy > 0))*resolution;
            ty = (boundary-a.y())/delta.y();
            dy = resolution/std::abs(delta.y());
        }
        // 逐格检查整段，不能用固定采样间隔跳过贴近边界的小段。
        while (index.head<2>() != end.head<2>()){
            if (std::abs(tx-ty) < 1e-12){
                if (!global_path_planner->isFree(index.x()+sx,index.y(),index.z()) ||
                    !global_path_planner->isFree(index.x(),index.y()+sy,index.z())) return false;
                index.x() += sx; index.y() += sy; tx += dx; ty += dy;
            }else if (tx < ty){ index.x() += sx; tx += dx; }
            else{ index.y() += sy; ty += dy; }
            if (!global_path_planner->isFree(index)) return false;
        }
        return true;
    }

    bool plan(Eigen::Vector3d start, Eigen::Vector3d goal, bool is_cross){
        local_optimizer->clear_traj();
        fallback_route.clear();
        search_route.clear();
        start.z() = 0.05;
        goal.z() = 0.05;
        if (is_cross) global_path_planner->setObsCross();
        else global_path_planner->resetObs();
        if (!isFree(start) || !isFree(goal)){ last_error = "occupied start/goal"; return false; }
        std::vector<Eigen::Vector3d> route;
        if (!global_path_planner->GetPath(route, start, goal)){ last_error = "A* failed"; return false; }
        if ((route.back()-goal).head<2>().norm() > 0.2){ last_error = "partial A* path"; return false; }
        route.insert(route.begin(), global_path_planner->gridIndex2coord(global_path_planner->coord2gridIndex(start)));
        route.insert(route.begin(), start);
        route.push_back(goal);
        for (auto &point : route) point.z() = 0.05;
        for (size_t i = 1; i < route.size(); i++){
            if (!segmentFree(route[i-1], route[i])){ last_error = "A* segment outside free map"; return false; }
        }
        search_route = route;
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
                // 用网格角点表示障碍格的范围，不能只检查障碍中心。
                double half_cell = global_path_planner->resolution_*0.5 + 0.01;
                for (double dx : {-half_cell, half_cell}){
                    for (double dy : {-half_cell, half_cell}){
                        nearby_pc.emplace_back(point.x()+dx, point.y()+dy, 0.025);
                        nearby_pc.emplace_back(point.x()+dx, point.y()+dy, 0.075);
                    }
                }
            }
        }
        // 平滑走廊也保留 A* 该段的地形选择，不能从低地切回邻近坡面。
        std::vector<double> route_costs;
        for (auto &point : route) route_costs.push_back(global_path_planner->traversalCost(point));
        auto low_index = global_path_planner->coord2gridIndex(low-Eigen::Vector3d(1,1,0));
        auto high_index = global_path_planner->coord2gridIndex(high+Eigen::Vector3d(1,1,0));
        for (int x = low_index.x(); x <= high_index.x(); x++){
            for (int y = low_index.y(); y <= high_index.y(); y++){
                Eigen::Vector3d point = global_path_planner->gridIndex2coord(Eigen::Vector3i(x,y,0));
                if (!isFree(point)) continue;
                size_t nearest = 0;
                double distance = INFINITY;
                for (size_t i = 0; i < route.size(); i++){
                    double d = (point-route[i]).head<2>().squaredNorm();
                    if (d < distance){ distance = d; nearest = i; }
                }
                // 保留路线两侧至少 0.25m 的走廊；额外限制只作用于更高代价地形。
                if (distance <= 0.25*0.25 || global_path_planner->traversalCost(point) <= route_costs[nearest]*1.5) continue;
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
        if (SFCs.empty()) return useFallback(route, "empty SFC");
        // 地面导航使用走廊在 z=0.05 的平面截面，避免三维优化从障碍柱上方绕过。
        for (auto &poly : SFCs){
            for (int i = 0; i < poly.rows(); i++){
                if (std::hypot(poly(i,0),poly(i,1)) > 1e-7){
                    poly(i,3) += 0.05*poly(i,2);
                    poly(i,2) = 0;
                }
            }
        }
        if (!local_optimizer->get_trajectory(route, SFCs, global_path_planner->esdf)) return useFallback(route, "MINCO failed");
        double route_cost = 0;
        for (size_t i = 1; i < route.size(); i++){
            route_cost += (route[i]-route[i-1]).norm()*global_path_planner->traversalCost((route[i]+route[i-1])*0.5);
        }
        double curve_cost = 0;
        // 优化成功后检查整条曲线，失败时不执行未经检查的路径。
        auto traj = local_optimizer->get_traj();
        Eigen::Vector3d last = start;
        int n = std::max(2, static_cast<int>(std::ceil(traj->getTotalDuration()/0.02)));
        n = std::min(n, 20000);
        for (int i = 1; i <= n; i++){
            Eigen::Vector3d point = traj->getPos(traj->getTotalDuration()*i/n);
            if (!point.allFinite() || !segmentFree(last, point)){
                last_error = "curve outside free map at " + std::to_string(point.x()) + "," + std::to_string(point.y()) + ",z=" + std::to_string(point.z());
                return useFallback(route, last_error);
            }
            size_t nearest = 0;
            double distance = INFINITY;
            for (size_t j = 0; j < route.size(); j++){
                double d = (point-route[j]).head<2>().squaredNorm();
                if (d < distance){ distance = d; nearest = j; }
            }
            if (distance > 0.25*0.25 && global_path_planner->traversalCost(point) > route_costs[nearest]*1.5){
                return useFallback(route, "curve leaves selected terrain");
            }
            curve_cost += (point-last).head<2>().norm()*global_path_planner->traversalCost((point+last)*0.5);
            last = point;
        }
        if (curve_cost > route_cost*1.2) return useFallback(route, "curve increases terrain cost");
        return true;
    }

    bool useFallback(const std::vector<Eigen::Vector3d> &route, const std::string &reason){
        local_optimizer->clear_traj();
        fallback_route = route;
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "Use checked A* path: %s", reason.c_str());
        return true;
    }

    Eigen::Vector3d getDirection(){
        if (fallback_route.empty()){
            auto direction = local_optimizer->get_v_directioin();
            if (direction.allFinite() && direction.norm() > 1e-6 &&
                segmentFree(search_route.front(),search_route.front()+direction.normalized()*0.1)) return direction;
            useFallback(search_route,"lookahead cuts outside free map");
        }
        Eigen::Vector3d direction = Eigen::Vector3d::Zero();
        for (auto &point : fallback_route){
            // 前瞻连线也必须可通行，不能跳过折点直接切入障碍格。
            if (!segmentFree(fallback_route.front(), point)) break;
            direction = point-fallback_route.front();
            if (direction.head<2>().norm() >= 0.1) break;
        }
        return direction;
    }

    nav_msgs::msg::Path getPath(const rclcpp::Time &stamp){
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = stamp;
        if (!fallback_route.empty()){
            for (size_t i = 1; i < fallback_route.size(); i++){
                auto a = fallback_route[i-1], b = fallback_route[i];
                int n = std::max(1, static_cast<int>(std::ceil((b-a).norm()/0.05)));
                for (int j = 0; j < n; j++){
                    auto point = a+(b-a)*static_cast<double>(j)/n;
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position.x = point.x();
                    pose.pose.position.y = point.y();
                    pose.pose.orientation.w = 1;
                    path.poses.push_back(pose);
                }
            }
            auto pose = path.poses.back();
            pose.pose.position.x = fallback_route.back().x();
            pose.pose.position.y = fallback_route.back().y();
            path.poses.push_back(pose);
            return path;
        }
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
