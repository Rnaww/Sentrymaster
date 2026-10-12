#pragma once
#include "common.h"
#include <nav_msgs/msg/path.hpp>

// 用相同的 701 高度图和速度预算约束两个控制器，单位 m/s。
class TerrainSpeed{
private:
    cv::Mat height_map;
    double resolution;
    std::vector<double> mapBound;
public:
    TerrainSpeed(rclcpp::Node *node){
        if (!node->has_parameter("MapBound")) node->declare_parameter<std::vector<double>>("MapBound", {-25,25,-25,25,0,0.1});
        if (!node->has_parameter("globalPlanner.resolution")) node->declare_parameter("globalPlanner.resolution", 0.1);
        if (!node->has_parameter("globalPlanner.PreDeepPath")) node->declare_parameter("globalPlanner.PreDeepPath", "");
        mapBound = node->get_parameter("MapBound").as_double_array();
        resolution = node->get_parameter("globalPlanner.resolution").as_double();
        height_map = cv::imread(node->get_parameter("globalPlanner.PreDeepPath").as_string(), 0);
        if (height_map.empty()) throw std::runtime_error("701 height map is empty");
    }

    double height(double x, double y){
        int i = static_cast<int>((x-mapBound[0])/resolution);
        int j = static_cast<int>((y-mapBound[2])/resolution);
        if (i < 0 || j < 0 || i >= height_map.rows || j >= height_map.cols) return 0;
        return height_map.at<uchar>(i,j)*0.8/255;
    }

    double terrainLimit(double x, double y){
        double h = height(x,y);
        double limit = (x > 14.2 && x < 16.2 && y > 2.2 && y < 6.8) ? 0.65 : 3.5;
        for (auto offset : {Eigen::Vector2d(0.3,0), Eigen::Vector2d(-0.3,0), Eigen::Vector2d(0,0.3), Eigen::Vector2d(0,-0.3)}){
            double other = height(x+offset.x(),y+offset.y());
            if (other >= 0.015 && other <= 0.41 && std::abs(other-h) > 0.015) limit = 0.45;
        }
        return limit;
    }

    double getLimit(const Eigen::Vector3d &position, const nav_msgs::msg::Path &path, double requested){
        double limit = std::min(requested, terrainLimit(position.x(),position.y()));
        if (path.poses.empty()) return limit;
        auto &goal = path.poses.back().pose.position;
        double remaining = std::hypot(goal.x-position.x(),goal.y-position.y());
        // 原末端 0.5m 减速不足以制停 3.5m/s；按同一 1.5m/s² 预算提前制动。
        limit = std::min(limit,std::sqrt(3.0*std::max(0.0,remaining-0.1)));
        size_t nearest = 0;
        double nearest_dis = INFINITY;
        for (size_t i = 0; i < path.poses.size(); i++){
            auto &p = path.poses[i].pose.position;
            double distance = std::hypot(p.x-position.x(),p.y-position.y());
            if (distance < nearest_dis){ nearest = i; nearest_dis = distance; }
        }
        std::vector<Eigen::Vector2d> points;
        std::vector<double> distances;
        points.emplace_back(position.x(),position.y());
        distances.push_back(0);
        double distance = 0;
        for (size_t i = nearest; i < path.poses.size(); i++){
            auto &p = path.poses[i].pose.position;
            Eigen::Vector2d point(p.x,p.y);
            double step = (point-points.back()).norm();
            if (step < 0.15 && i+1 < path.poses.size()) continue;
            distance += step;
            points.push_back(point);
            distances.push_back(distance);
            double terrain = terrainLimit(p.x,p.y);
            limit = std::min(limit, std::sqrt(terrain*terrain+3.0*std::max(0.0,distance-0.4)));
            if (points.size() >= 3){
                auto a = points[points.size()-3], b = points[points.size()-2], c = points.back();
                auto ab = b-a, ac = c-a;
                double denominator = ab.norm()*ac.norm()*(c-b).norm();
                if (denominator > 1e-6){
                    double curvature = 2*std::abs(ab.x()*ac.y()-ab.y()*ac.x())/denominator;
                    if (curvature > 1e-5){
                        double curve_speed = std::sqrt(0.35/curvature);
                        double ahead = std::max(0.0,distances[distances.size()-2]-0.4);
                        limit = std::min(limit,std::sqrt(curve_speed*curve_speed+3.0*ahead));
                    }
                }
            }
            if (distance > 5) break;
        }
        return std::max(0.0,limit);
    }
};
