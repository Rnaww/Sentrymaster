#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <chrono>
#include <cmath>
#include <mutex>

namespace sentry_gazebo{
// 只接管平面运动，保留重力、垂直速度和坡面接触产生的姿态变化。
class PlanarMove : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate{
public:
    void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override{
        gz::sim::Model model(entity);
        link = gz::sim::Link(model.CanonicalLink(ecm));
        link.EnableVelocityChecks(ecm);
        std::string topic = sdf->Get<std::string>("topic", "/sentry_robot/cmd_vel").first;
        transport.Subscribe(topic, &PlanarMove::commandCallback, this);
    }

    void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override{
        if (info.paused || !link.Valid(ecm))
            return;
        gz::msgs::Twist command;
        {
            std::lock_guard<std::mutex> lock(command_mutex);
            if (received && std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command).count() < 0.5)
                command = target;
        }
        auto pose = gz::sim::worldPose(link.Entity(), ecm);
        double yaw = pose.Rot().Yaw();
        auto linear = link.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
        auto angular = link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
        linear.X(std::cos(yaw) * command.linear().x() - std::sin(yaw) * command.linear().y());
        linear.Y(std::sin(yaw) * command.linear().x() + std::cos(yaw) * command.linear().y());
        angular.Z(command.angular().z());
        // Gazebo Link 的速度接口使用 link 系，导航命令仍是右手车体系。
        link.SetLinearVelocity(ecm, pose.Rot().RotateVectorReverse(linear));
        link.SetAngularVelocity(ecm, pose.Rot().RotateVectorReverse(angular));
    }

private:
    void commandCallback(const gz::msgs::Twist &message){
        if (!std::isfinite(message.linear().x()) || !std::isfinite(message.linear().y()) ||
            !std::isfinite(message.angular().z()))
            return;
        std::lock_guard<std::mutex> lock(command_mutex);
        target = message;
        last_command = std::chrono::steady_clock::now();
        received = true;
    }

    gz::sim::Link link;
    gz::transport::Node transport;
    gz::msgs::Twist target;
    std::mutex command_mutex;
    std::chrono::steady_clock::time_point last_command;
    bool received = false;
};
}
GZ_ADD_PLUGIN(sentry_gazebo::PlanarMove, gz::sim::System,
    sentry_gazebo::PlanarMove::ISystemConfigure, sentry_gazebo::PlanarMove::ISystemPreUpdate)
