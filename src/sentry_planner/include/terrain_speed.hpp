#pragma once
#include "common.h"
#include <nav_msgs/msg/path.hpp>

// 用相同的 701 高度图和速度预算约束两个控制器，单位 m/s。
class TerrainSpeed{
private:
    cv::Mat height_map;
    cv::Mat clearance_map;
    rclcpp::Node *node;
    double uphill_speed, rough_speed, lateral_acc;
    Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
    double max_acc, last_speed_time = -1;
    double resolution;
    std::vector<double> mapBound;
public:
    TerrainSpeed(rclcpp::Node *node):node(node){
        uphill_speed = node->declare_parameter("terrainSpeed.uphill_speed", 2.0);
        rough_speed = node->declare_parameter("terrainSpeed.rough_speed", 2.0);
        lateral_acc = node->declare_parameter("terrainSpeed.lateral_acc", 1.0);
        if (uphill_speed <= 0 || rough_speed <= 0 || lateral_acc <= 0 ||
            !std::isfinite(uphill_speed+rough_speed+lateral_acc)) throw std::runtime_error("terrain speed parameters must be positive");
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
        info_log("TERRAIN YAML: uphill_speed=", uphill_speed, " rough_speed=", rough_speed,
                 " lateral_acc=", lateral_acc, " planar_max_acc=", max_acc);
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

    Eigen::Vector3d checkedVelocity(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity){
        double speed = velocity.head<2>().norm();
        if (speed < 1e-6) return Eigen::Vector3d::Zero();
        double distance = std::max(0.1,speed*0.1+speed*speed/(2*max_acc));
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
        double limit = safe_distance >= distance-1e-6 ? speed : std::max(0.0,std::sqrt(max_acc*max_acc*0.01+2*max_acc*std::max(0.0,safe_distance-0.02))-max_acc*0.1);
        return velocity*std::min(1.0,limit/speed);
    }

    double height(double x, double y){
        int i = static_cast<int>((x-mapBound[0])/resolution);
        int j = static_cast<int>((y-mapBound[2])/resolution);
        if (i < 0 || j < 0 || i >= height_map.rows || j >= height_map.cols) return 0;
        return height_map.at<uchar>(i,j)*0.8/255;
    }

    bool onRoughTerrain(double x, double y){
        return x > 14.2 && x < 16.2 && y > 2.2 && y < 6.8;
    }

    double terrainLimit(double x, double y){
        return onRoughTerrain(x,y) ? rough_speed : 4.5;
    }

    Eigen::Vector3d limitVelocity(const Eigen::Vector3d &velocity, double time, bool emergency=false){
        // 使用实际控制间隔，不能把较慢回调截成 0.05s 后仍按完整加速度估算制动。
        double dt = last_speed_time < 0 ? 0.01 : std::clamp(time-last_speed_time,0.0,0.5);
        last_speed_time = time;
        if (emergency){ previous_velocity.setZero(); return previous_velocity; }
        Eigen::Vector3d change = velocity-previous_velocity;
        if (change.norm() > max_acc*dt && change.norm() > 1e-9) change *= max_acc*dt/change.norm();
        previous_velocity += change;
        return previous_velocity;
    }

    std::vector<double> speedProfile(const std::vector<Eigen::Vector3d> &points, const std::vector<double> &distances, double requested, bool terminal){
        std::vector<double> speeds(points.size(),requested);
        for (size_t i = 0; i < points.size(); i++){
            speeds[i] = std::min(speeds[i],terrainLimit(points[i].x(),points[i].y()));
            size_t a = std::lower_bound(distances.begin(),distances.end(),std::max(0.0,distances[i]-0.3))-distances.begin();
            size_t b = std::min(static_cast<size_t>(std::lower_bound(distances.begin(),distances.end(),distances[i]+0.3)-distances.begin()),points.size()-1);
            // 只对沿行驶方向升高的路段使用上坡速度；下坡不另设限速。
            if (!onRoughTerrain(points[i].x(),points[i].y()) &&
                height(points[b].x(),points[b].y())-height(points[a].x(),points[a].y()) > 0.01){
                speeds[i] = std::min(speeds[i],uphill_speed);
            }
            if (a < i && i < b){
                auto ab = points[i]-points[a], ac = points[b]-points[a];
                double denominator = ab.head<2>().norm()*ac.head<2>().norm()*(points[b]-points[i]).head<2>().norm();
                if (denominator > 1e-6){
                    double curvature = 2*std::abs(ab.x()*ac.y()-ab.y()*ac.x())/denominator;
                    if (curvature > 1e-5) speeds[i] = std::min(speeds[i],std::sqrt(lateral_acc/curvature));
                }
            }
        }
        if (terminal) speeds.back() = 0;
        for (size_t i = speeds.size()-1; i > 0; i--){
            speeds[i-1] = std::min(speeds[i-1],std::sqrt(speeds[i]*speeds[i]+2*max_acc*(distances[i]-distances[i-1])));
        }
        return speeds;
    }

    void acceptVelocity(const Eigen::Vector3d &velocity){ previous_velocity = velocity; }

    double getLimit(const Eigen::Vector3d &position, const nav_msgs::msg::Path &path, double requested){
        double cap = std::min(requested,terrainLimit(position.x(),position.y()));
        if (path.poses.empty()) return std::max(0.0,cap);
        size_t nearest = 0;
        double nearest_dis = INFINITY;
        Eigen::Vector3d projection = position;
        // 曲率沿规划路径计算，车体的小幅侧向误差不是路径的急弯。
        for (size_t i = 1; i < path.poses.size(); i++){
            auto &a = path.poses[i-1].pose.position, &b = path.poses[i].pose.position;
            Eigen::Vector3d start(a.x,a.y,position.z()), end(b.x,b.y,position.z());
            auto delta = end-start;
            double ratio = delta.squaredNorm() > 1e-12 ? std::clamp((position-start).dot(delta)/delta.squaredNorm(),0.0,1.0) : 0;
            Eigen::Vector3d point = start+delta*ratio;
            double distance = (point-position).squaredNorm();
            if (distance < nearest_dis){ nearest = i; nearest_dis = distance; projection = point; }
        }
        std::vector<Eigen::Vector3d> points{projection};
        std::vector<double> distances{0};
        double distance = 0;
        bool terminal = true;
        for (size_t i = nearest; i < path.poses.size(); i++){
            auto &p = path.poses[i].pose.position;
            Eigen::Vector3d target(p.x,p.y,position.z());
            Eigen::Vector3d begin = points.back();
            double length = (target-begin).head<2>().norm();
            if (length < 1e-6) continue;
            int n = std::max(1,static_cast<int>(std::ceil(length/0.1)));
            for (int j = 1; j <= n; j++){
                auto point = begin+(target-begin)*static_cast<double>(j)/n;
                distance += (point-points.back()).head<2>().norm();
                points.push_back(point); distances.push_back(distance);
            }
            if (distance >= 5 && i+1 < path.poses.size()){ terminal = false; break; }
        }
        if (points.size() < 2){
            auto &goal = path.poses.back().pose.position;
            Eigen::Vector3d target(goal.x,goal.y,position.z());
            double remaining = (target-position).norm();
            if (remaining < 1e-6) return 0;
            points = {position,target}; distances = {0,remaining};
        }
        auto speeds = speedProfile(points,distances,requested,terminal);
        cap = std::min(cap,speeds.front());
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                            "SPEED: requested=%.2f allowed=%.2f m/s, uphill=%.2f rough=%.2f lateral_acc=%.2f",
                            requested, cap, uphill_speed, rough_speed, lateral_acc);
        return std::max(0.0,cap);
    }
};
