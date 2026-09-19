#include <memory>

#include <std_srvs/srv/set_bool.hpp>

#include "mujoco_ros2_plugins/mujoco_ros2_plugins.hpp"

namespace mujoco_ros2_plugins
{

class EmergencyStopPlugin : public PluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * /*data*/) override
  {
    node_ = node;
    model_ = model;
    // Absolute name, the same one sts_hardware_interface serves on the real robot.
    service_ = node->create_service<std_srvs::srv::SetBool>(
      "/emergency_stop",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
        stop_.set_active(request->data);
        response->success = true;
        response->message = request->data ? "Emergency stop enabled" : "Emergency stop disabled";
        if (request->data) {
          RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
        } else {
          RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
        }
      });
    return true;
  }

  void pre_step(mjData * data) override {stop_.apply(model_, data);}

  void on_reset(mjData * /*data*/) override {stop_.reset_latch();}

  void cleanup() override {service_.reset();}

private:
  rclcpp::Node::SharedPtr node_;
  const mjModel * model_{nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  EmergencyStop stop_;
};

}  // namespace mujoco_ros2_plugins

MUJOCO_ROS2_PLUGINS_EXPORT(mujoco_ros2_plugins::EmergencyStopPlugin)
