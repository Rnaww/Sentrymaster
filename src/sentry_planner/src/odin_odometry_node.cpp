#include "common.h"
#include <tf2_ros/static_transform_broadcaster.h>

// 为两个导航分支提供同一车体里程计和 TF。
class OdinOdometryNode : public rclcpp::Node{
public:
    OdinOdometryNode():Node("odin_odometry_node"){
        initial_pose = declare_parameter<std::vector<double>>("initial_pose", {7.25, 2.70, 0.0});
        align_initial_pose = declare_parameter("align_initial_pose", true);
        use_odin_map_tf = declare_parameter("use_odin_map_tf", false);
        if (initial_pose.size() != 3) throw std::runtime_error("initial_pose needs x, y, yaw");
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_puber = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_puber = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        odom_puber = create_publisher<OdomMsg>("/navigation_odom", 10);
        odom_suber = create_subscription<OdomMsg>("/odin1/odometry_highfreq", 10, std::bind(&OdinOdometryNode::odomCallback, this, std::placeholders::_1));
        info_log("ODOM YAML: align_initial_pose=", align_initial_pose, " use_odin_map_tf=", use_odin_map_tf,
                 " initial_pose=", initial_pose[0], initial_pose[1], initial_pose[2]);
    }
private:
    std::vector<double> initial_pose;
    bool align_initial_pose, use_odin_map_tf, initialized = false;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_puber;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_puber;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_suber;
    rclcpp::Publisher<OdomMsg>::SharedPtr odom_puber;
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
    double last_stamp = 0, last_yaw = 0;

    void odomCallback(OdomMsg::SharedPtr msg){
        auto &q = msg->pose.pose.orientation;
        Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6) return;
        Eigen::Affine3d odom_odin = Eigen::Affine3d::Identity();
        odom_odin.linear() = rotation.normalized().toRotationMatrix();
        auto &p = msg->pose.pose.position;
        odom_odin.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        if (!odom_odin.translation().allFinite() || msg->header.frame_id != "odom") return;
        Eigen::Affine3d odom_vehicle;
        try{
            auto fixed = tf_buffer->lookupTransform(msg->child_frame_id, "vehicle", tf2::TimePointZero);
            odom_vehicle = odom_odin * tf2::transformToEigen(fixed.transform);
        }catch(const tf2::TransformException &ex){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "vehicle TF not ready: %s", ex.what());
            return;
        }
        double stamp = rclcpp::Time(msg->header.stamp).seconds();
        if (initialized && stamp <= last_stamp) return;
        if (!initialized && !use_odin_map_tf){
            Eigen::Affine3d map_odom = Eigen::Affine3d::Identity();
            if (align_initial_pose){
                Eigen::Affine3d map_vehicle = Eigen::Affine3d::Identity();
                map_vehicle.linear() = Eigen::AngleAxisd(initial_pose[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
                map_vehicle.translation() = Eigen::Vector3d(initial_pose[0], initial_pose[1], 0);
                map_odom = map_vehicle * odom_vehicle.inverse();
            }
            auto map_tf = tf2::eigenToTransform(map_odom);
            map_tf.header.stamp = msg->header.stamp;
            map_tf.header.frame_id = "map";
            map_tf.child_frame_id = "odom";
            static_tf_puber->sendTransform(map_tf);
        }
        auto odin_tf = tf2::eigenToTransform(odom_odin);
        odin_tf.header = msg->header;
        odin_tf.child_frame_id = msg->child_frame_id;
        tf_puber->sendTransform(odin_tf);
        OdomMsg output;
        output.header = msg->header;
        output.child_frame_id = "vehicle";
        auto vehicle_tf = tf2::eigenToTransform(odom_vehicle);
        output.pose.pose.position.x = vehicle_tf.transform.translation.x;
        output.pose.pose.position.y = vehicle_tf.transform.translation.y;
        output.pose.pose.position.z = vehicle_tf.transform.translation.z;
        output.pose.pose.orientation = vehicle_tf.transform.rotation;
        double yaw = std::atan2(odom_vehicle.linear()(1,0), odom_vehicle.linear()(0,0));
        double dt = stamp-last_stamp;
        if (initialized && dt > 0 && dt < 0.5){
            Eigen::Vector3d v = odom_vehicle.linear().transpose() * ((odom_vehicle.translation()-last_position)/dt);
            output.twist.twist.linear.x = v.x();
            output.twist.twist.linear.y = v.y();
            output.twist.twist.angular.z = std::atan2(std::sin(yaw-last_yaw), std::cos(yaw-last_yaw))/dt;
        }
        initialized = true;
        last_position = odom_vehicle.translation();
        last_yaw = yaw;
        last_stamp = stamp;
        odom_puber->publish(output);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdinOdometryNode>());
    rclcpp::shutdown();
}
