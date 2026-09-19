#ifndef MUJOCO_ROS2_PLUGINS__MUJOCO_ROS2_PLUGINS_HPP_
#define MUJOCO_ROS2_PLUGINS__MUJOCO_ROS2_PLUGINS_HPP_

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <vector>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// The one header of this package: the plugin base class, the export macro and the ROS-free
// cores of the plugins (testable with plain MuJoCo).
namespace mujoco_ros2_plugins
{

using PluginBase = mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase;

// ROS-free core of the emergency stop, so it can be tested with plain MuJoCo. Robot-agnostic.
class EmergencyStop
{
public:
  // Any thread.
  void set_active(bool active) {active_.store(active);}
  bool active() const {return active_.load();}
  // Physics thread: call after a world reset so the stop is latched again from the new state.
  void reset_latch() {latched_ = false;}

  // Physics thread, immediately before every mj_step().
  void apply(const mjModel * model, mjData * data)
  {
    if (!active_.load()) {
      latched_ = false;
      return;
    }
    if (!latched_) {
      held_.assign(model->nu, 0.0);
      for (int i = 0; i < model->nu; ++i) {
        if (holds_position(model, i)) {
          double q = data->qpos[model->jnt_qposadr[model->actuator_trnid[2 * i]]];
          if (model->actuator_ctrllimited[i]) {
            q = std::clamp(q, model->actuator_ctrlrange[2 * i], model->actuator_ctrlrange[2 * i + 1]);
          }
          held_[i] = q;
        }
      }
      latched_ = true;
    }
    for (int i = 0; i < model->nu; ++i) {
      data->ctrl[i] = held_[i];
    }
  }

  // A joint-driven position servo (affine bias on position) must hold its angle; zero would
  // command it to 0 rad. Everything else, e.g. the wheels' velocity servos, is commanded to zero.
  static bool holds_position(const mjModel * model, int actuator)
  {
    return model->actuator_trntype[actuator] == mjTRN_JOINT &&
           model->actuator_biastype[actuator] == mjBIAS_AFFINE &&
           model->actuator_biasprm[mjNBIAS * actuator + 1] != 0.0;
  }

private:
  std::atomic<bool> active_{false};
  bool latched_{false};
  std::vector<double> held_;
};

}  // namespace mujoco_ros2_plugins

// Registers a plugin class under this package's loader base class.
#define MUJOCO_ROS2_PLUGINS_EXPORT(plugin_class) \
  PLUGINLIB_EXPORT_CLASS(plugin_class, mujoco_ros2_plugins::PluginBase)

#endif  // MUJOCO_ROS2_PLUGINS__MUJOCO_ROS2_PLUGINS_HPP_
