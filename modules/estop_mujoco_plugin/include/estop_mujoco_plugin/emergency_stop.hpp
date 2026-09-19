#ifndef ESTOP_MUJOCO_PLUGIN__EMERGENCY_STOP_HPP_
#define ESTOP_MUJOCO_PLUGIN__EMERGENCY_STOP_HPP_

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <vector>

namespace estop_mujoco_plugin
{

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

}  // namespace estop_mujoco_plugin

#endif  // ESTOP_MUJOCO_PLUGIN__EMERGENCY_STOP_HPP_
