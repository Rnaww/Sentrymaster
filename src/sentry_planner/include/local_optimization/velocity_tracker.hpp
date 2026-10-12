#pragma once
#include "common.h"

// 末端速度跟踪器
class VelocityTracker{
private:
    // 优化后的速度和加速度
    Eigen::Vector3d last_a, last_v;
    double dt, tracker_coef;

    // 计算变速小陀螺的转速
    double start_time, omega, offset, size, slow_offset, slow_size;

    // 正常导航速度平滑器
    Eigen::Vector3d last_nav_v;
    double last_nav_time;
    double planar_max_acc, planar_lpf_hz, planar_stop_speed;
    bool nav_v_inited = false;
    
    // 高斯滤波器参数
    std::vector<Eigen::Vector3d> chassis_vs; 
    std::vector<double> kernels;
    Eigen::Vector3d last_opt_v;
    int window_size;
    double sigma, kernels_sum, pre_hz;
    
public:
    VelocityTracker(rclcpp::Node *node){
        dt = node->declare_parameter("velocityTracker.dt", 0.0);
        tracker_coef = node->declare_parameter("velocityTracker.tracker_coef", 0.0);

        omega = node->declare_parameter("velocityTracker.omega", 0.0);
        offset = node->declare_parameter("velocityTracker.offset", 0.0);
        size = node->declare_parameter("velocityTracker.size", 0.0);
        slow_offset = node->declare_parameter("velocityTracker.slow_offset", 0.0);
        slow_size = node->declare_parameter("velocityTracker.slow_size", 0.0);

        planar_max_acc = node->declare_parameter("velocityTracker.planar_max_acc", 0.0);
        planar_lpf_hz = node->declare_parameter("velocityTracker.planar_lpf_hz", 0.0);
        planar_stop_speed = node->declare_parameter("velocityTracker.planar_stop_speed", 0.0);
        last_nav_v.setZero();
        last_nav_time = node->now().seconds();

        sigma = node->declare_parameter("velocityTracker.sigma", 0.0);
        window_size = node->declare_parameter("velocityTracker.window_size", 0);
        pre_hz = node->declare_parameter("velocityTracker.pre_hz", 0.0);
        kernels_sum = 1.0;
        for (int i = 0; i < window_size; i++){
            double kernal = exp(-i*i/(2*sigma*sigma));
            kernels_sum += kernal;
            kernels.push_back(kernal);
        }

        start_time = node->now().seconds();
        info_log("velocity tracker config: dt:", dt, "  tracker_coef:", tracker_coef);
    }

    double sign(double num) {
        const double eps = 1e-12;
        if (num > eps) {
            return 1.0;
        } else if (num < -eps) {
            return -1.0;
        } else {
            return 0.0;
        }
    }

    // 得到当前变速小陀螺的速度
    double get_wz(double current_time){
        dt = current_time - start_time;
        return size * sin(omega * dt) + offset;
    }

    // 得到当前慢速变速小陀螺的速度
    double get_slow_wz(double current_time){
        dt = current_time - start_time;
        return slow_size * sin(omega * dt) + slow_offset;
    }

    Eigen::Vector3d get_nav_optimize_v(const Eigen::Vector3d &aim_v, const Eigen::Vector3d &current_v, double current_time){
        Eigen::Vector3d target_v = aim_v;
        target_v.z() = 0.0;
        Eigen::Vector3d feedback_v = current_v;
        feedback_v.z() = 0.0;

        if (!nav_v_inited || current_time - last_nav_time > 0.1) {
            last_nav_v = feedback_v;
            last_nav_time = current_time;
            nav_v_inited = true;
        }

        double sample_dt = current_time - last_nav_time;
        if (sample_dt <= 1e-4) {
            return last_nav_v;
        }
        last_nav_time = current_time;

        Eigen::Vector2d delta_v = (target_v - last_nav_v).head<2>();
        const double delta_norm = delta_v.norm();
        const double max_delta_v = planar_max_acc * sample_dt;
        Eigen::Vector3d rate_limited_v = target_v;
        if (planar_max_acc > 0.0 && delta_norm > max_delta_v && delta_norm > 1e-6) {
            rate_limited_v.head<2>() = last_nav_v.head<2>() + delta_v * (max_delta_v / delta_norm);
        }

        if (planar_lpf_hz > 1e-6) {
            const double tau = 1.0 / (2.0 * M_PI * planar_lpf_hz);
            double alpha = sample_dt / (tau + sample_dt);
            if (alpha < 0.0) {
                alpha = 0.0;
            } else if (alpha > 1.0) {
                alpha = 1.0;
            }
            last_nav_v += alpha * (rate_limited_v - last_nav_v);
        } else {
            last_nav_v = rate_limited_v;
        }

        if (last_nav_v.head<2>().norm() < planar_stop_speed) {
            last_nav_v.head<2>().setZero();
        }
        last_nav_v.z() = 0.0;
        return last_nav_v;
    }

    // 得到dt滤波后的底盘速度
    Eigen::Vector3d get_optimize_v(const Eigen::Vector3d &aim_v){
        Eigen::Vector3d fh = Eigen::Vector3d::Zero();
        const double d = tracker_coef * dt * dt;
        for (int i = 0; i < 2; ++i) {
            const double y = (last_a(i) - aim_v(i)) + dt * last_v(i);

            double a;
            if (std::fabs(y) > d) {
                const double a1 = std::sqrt(d * (d + 8.0 * std::fabs(y)));
                a = last_v(i) + (a1 - d) * sign(y) / (2.0 * dt);
            } else {
                a = last_v(i) + y / dt;
            }

            if (std::fabs(a) > tracker_coef * dt) {
                fh(i) = -tracker_coef * sign(a);
            } else {
                fh(i) = -a / dt;
            }
        }

        last_a += dt * last_v;
        last_v += dt * fh;
        return last_a;
    }

    // 得到小陀螺移动模式下滤波后的底盘速度
    Eigen::Vector3d get_rotate_optimize_v(const Eigen::Vector3d &aim_v){
        if (chassis_vs.size() < window_size){
            chassis_vs.push_back(aim_v);
            last_opt_v = aim_v;
            return aim_v;
        }else{
            // 高斯滤波
            Eigen::Vector3d opt_chassis_v = Eigen::Vector3d::Zero();
            for (int i = 0; i < 2; i++){
                double opt_velocity_size = aim_v[i];
                for (int p = 0; p < window_size; p++){
                    opt_velocity_size += kernels[p] * chassis_vs[window_size - i - 1][i];
                }
                opt_velocity_size /= kernels_sum;
                opt_chassis_v[i] = opt_velocity_size;
            }
            chassis_vs.push_back(aim_v);
            chassis_vs.erase(chassis_vs.begin());

            // 低通滤波
            double tau = 1.0 / (2.0 * M_PI * pre_hz);
            double alpha = dt / (tau + dt);
            last_opt_v = last_opt_v + alpha * (opt_chassis_v - last_opt_v);
            return last_opt_v;
        }
    }
};
