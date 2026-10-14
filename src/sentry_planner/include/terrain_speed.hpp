#pragma once
#include "common.h"
#include <nav_msgs/msg/path.hpp>

// 用相同的 701 高度图和速度预算约束两个控制器，单位 m/s。
class TerrainSpeed{
private:
    cv::Mat height_map;
    cv::Mat clearance_map;
    cv::Mat terrain_speed_map;
    double max_acc, previous_speed = 0, last_speed_time = -1;
    double resolution;
    std::vector<double> mapBound;
public:
    TerrainSpeed(rclcpp::Node *node){
        if (!node->has_parameter("MapBound")) node->declare_parameter<std::vector<double>>("MapBound", {-25,25,-25,25,0,0.1});
        if (!node->has_parameter("globalPlanner.resolution")) node->declare_parameter("globalPlanner.resolution", 0.1);
        if (!node->has_parameter("globalPlanner.PreDeepPath")) node->declare_parameter("globalPlanner.PreDeepPath", "");
        if (!node->has_parameter("velocityTracker.planar_max_acc")) node->declare_parameter("velocityTracker.planar_max_acc", 1500.0);
        max_acc = node->get_parameter("velocityTracker.planar_max_acc").as_double()/1000;
        if (!std::isfinite(max_acc) || max_acc <= 0) throw std::runtime_error("planar_max_acc must be positive");
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
        // 旋转包络约 0.424m，加栅格误差后按半径 0.5m 检查整个车体地形。
        cv::Mat low, high;
        auto footprint = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11));
        cv::erode(heights,low,footprint);
        cv::dilate(heights,high,footprint);
        terrain_speed_map = cv::Mat(heights.size(),CV_64F);
        for (int i = 0; i < heights.rows; i++){
            for (int j = 0; j < heights.cols; j++){
                double x = mapBound[0]+(i+0.5)*resolution, y = mapBound[2]+(j+0.5)*resolution;
                double limit = (x > 14.2 && x < 16.2 && y > 2.2 && y < 6.8) ? 0.65 : 4.5;
                if (high.at<double>(i,j)-low.at<double>(i,j) > 0.015) limit = 0.45;
                terrain_speed_map.at<double>(i,j) = limit;
            }
        }
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
        auto direction = velocity/speed;
        auto end = position+direction*distance;
        int row = static_cast<int>((position.x()-mapBound[0])/resolution);
        int col = static_cast<int>((position.y()-mapBound[2])/resolution);
        int end_row = static_cast<int>((end.x()-mapBound[0])/resolution);
        int end_col = static_cast<int>((end.y()-mapBound[2])/resolution);
        auto free_cell = [&](int r, int c){
            return r >= 0 && c >= 0 && r < clearance_map.rows && c < clearance_map.cols &&
                   clearance_map.at<float>(r,c) >= 0.6-1e-7;
        };
        if (!free_cell(row,col)) return Eigen::Vector3d::Zero();
        int sx = direction.x() > 0 ? 1 : -1, sy = direction.y() > 0 ? 1 : -1;
        double tx = INFINITY, ty = INFINITY, step_x = INFINITY, step_y = INFINITY;
        if (row != end_row){
            tx = (mapBound[0]+(row+(sx > 0))*resolution-position.x())/direction.x();
            step_x = resolution/std::abs(direction.x());
        }
        if (col != end_col){
            ty = (mapBound[2]+(col+(sy > 0))*resolution-position.y())/direction.y();
            step_y = resolution/std::abs(direction.y());
        }
        // 逐格检查，固定间隔采样会漏掉栅格转角处很短的不可通行段。
        double safe_distance = distance;
        while (row != end_row || col != end_col){
            double crossing = std::min(tx,ty);
            if (std::abs(tx-ty) < 1e-12){
                if (!free_cell(row+sx,col) || !free_cell(row,col+sy)){
                    safe_distance = crossing;
                    break;
                }
                row += sx; col += sy; tx += step_x; ty += step_y;
            }else if (tx < ty){ row += sx; tx += step_x; }
            else{ col += sy; ty += step_y; }
            if (!free_cell(row,col)){ safe_distance = crossing; break; }
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
        int i = static_cast<int>((x-mapBound[0])/resolution);
        int j = static_cast<int>((y-mapBound[2])/resolution);
        if (i < 0 || j < 0 || i >= terrain_speed_map.rows || j >= terrain_speed_map.cols) return 0;
        return terrain_speed_map.at<double>(i,j);
    }

    double limitSpeedRise(double speed, double time){
        double dt = last_speed_time < 0 ? 0.01 : std::clamp(time-last_speed_time,0.0,0.05);
        last_speed_time = time;
        // 增速按加速度预算执行；安全限速和停车仍立即生效。
        previous_speed = std::min(speed,previous_speed+max_acc*dt);
        return previous_speed;
    }

    double getLimit(const Eigen::Vector3d &position, const nav_msgs::msg::Path &path, double requested){
        double limit = std::min(requested, terrainLimit(position.x(),position.y()));
        if (path.poses.empty()) return limit;
        auto &goal = path.poses.back().pose.position;
        double remaining = std::hypot(goal.x-position.x(),goal.y-position.y());
        // 按 YAML 的平移加速度预算提前制动。
        limit = std::min(limit,std::sqrt(2*max_acc*std::max(0.0,remaining-0.01)));
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
            limit = std::min(limit, std::sqrt(terrain*terrain+2*max_acc*std::max(0.0,distance-0.4)));
            if (points.size() >= 3){
                auto a = points[points.size()-3], b = points[points.size()-2], c = points.back();
                auto ab = b-a, ac = c-a;
                double denominator = ab.norm()*ac.norm()*(c-b).norm();
                if (denominator > 1e-6){
                    double curvature = 2*std::abs(ab.x()*ac.y()-ab.y()*ac.x())/denominator;
                    if (curvature > 1e-5){
                        double curve_speed = std::sqrt(0.35/curvature);
                        double ahead = std::max(0.0,distances[distances.size()-2]-0.4);
                        limit = std::min(limit,std::sqrt(curve_speed*curve_speed+2*max_acc*ahead));
                    }
                }
            }
            if (distance > 5) break;
        }
        return std::max(0.0,limit);
    }
};
