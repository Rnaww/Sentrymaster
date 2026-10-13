#pragma once
#include "common.h"
#include <nav_msgs/msg/path.hpp>

// 用相同的 701 高度图和速度预算约束两个控制器，单位 m/s。
class TerrainSpeed{
private:
    cv::Mat height_map;
    cv::Mat clearance_map;
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
        cv::Mat heights, valid(height_map.size(), CV_8U, cv::Scalar(255));
        height_map.convertTo(heights, CV_64F, 0.8/255);
        for (int i = 0; i < heights.rows; i++){
            for (int j = 0; j < heights.cols; j++){
                double h = heights.at<double>(i,j);
                if (h < 0.015 || h > 0.41) valid.at<uchar>(i,j) = 0;
                for (auto offset : {Eigen::Vector2i(1,0), Eigen::Vector2i(0,1)}){
                    int r = i+offset.x(), c = j+offset.y();
                    if (r < heights.rows && c < heights.cols && std::abs(h-heights.at<double>(r,c)) > 0.055){
                        valid.at<uchar>(i,j) = 0;
                        valid.at<uchar>(r,c) = 0;
                    }
                }
            }
        }
        cv::distanceTransform(valid, clearance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        clearance_map *= resolution;
    }

    double clearance(double x, double y){
        int i = static_cast<int>((x-mapBound[0])/resolution);
        int j = static_cast<int>((y-mapBound[2])/resolution);
        if (i < 0 || j < 0 || i >= clearance_map.rows || j >= clearance_map.cols) return 0;
        return clearance_map.at<float>(i,j);
    }

    bool navigationFree(const Eigen::Vector3d &point){
        return clearance(point.x(),point.y()) >= 0.6-1e-7;
    }

    Eigen::Vector3d recoveryVelocity(const Eigen::Vector3d &position){
        Eigen::Vector3d target = position;
        double nearest = 0.3;
        double initial = clearance(position.x(),position.y());
        // 只恢复进入安全余量的情况；旋转车体包络加栅格误差仍须可容纳。
        if (initial < std::hypot(0.3,0.3)+resolution/std::sqrt(2.0)) return Eigen::Vector3d::Zero();
        int row = static_cast<int>((position.x()-mapBound[0])/resolution);
        int col = static_cast<int>((position.y()-mapBound[2])/resolution);
        for (int i = row-3; i <= row+3; i++){
            for (int j = col-3; j <= col+3; j++){
                Eigen::Vector3d point(mapBound[0]+(i+0.5)*resolution,mapBound[2]+(j+0.5)*resolution,position.z());
                double distance = (point-position).head<2>().norm();
                if (distance >= nearest || !navigationFree(point)) continue;
                bool safe = true;
                int n = std::max(1,static_cast<int>(std::ceil(distance/0.01)));
                for (int k = 0; k <= n; k++){
                    auto sample = position+(point-position)*static_cast<double>(k)/n;
                    if (clearance(sample.x(),sample.y()) < initial-1e-7){ safe = false; break; }
                }
                if (safe){ nearest = distance; target = point; }
            }
        }
        auto direction = target-position;
        return direction.norm() > 1e-6 ? Eigen::Vector3d(direction.normalized()*0.15) : Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d checkedVelocity(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity){
        double speed = velocity.head<2>().norm();
        if (speed < 1e-6) return Eigen::Vector3d::Zero();
        double distance = std::max(0.1,speed*0.1);
        int n = std::max(1,static_cast<int>(std::ceil(distance/0.01)));
        double safe_distance = 0;
        for (int i = 1; i <= n; i++){
            double step = distance*i/n;
            if (!navigationFree(position+velocity/speed*step)) break;
            safe_distance = step;
        }
        double limit = safe_distance >= distance-1e-6 ? speed : std::max(0.0,(safe_distance-0.02)/0.1);
        return velocity*std::min(1.0,limit/speed);
    }

    double height(double x, double y){
        int i = static_cast<int>((x-mapBound[0])/resolution);
        int j = static_cast<int>((y-mapBound[2])/resolution);
        if (i < 0 || j < 0 || i >= height_map.rows || j >= height_map.cols) return 0;
        return height_map.at<uchar>(i,j)*0.8/255;
    }

    double terrainLimit(double x, double y){
        double h = height(x,y);
        double limit = (x > 14.2 && x < 16.2 && y > 2.2 && y < 6.8) ? 0.65 : 4.5;
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
        // 按 1.5m/s² 预算提前制动。
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
