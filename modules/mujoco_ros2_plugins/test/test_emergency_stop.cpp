#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cmath>

#include "mujoco_ros2_plugins/mujoco_ros2_plugins.hpp"

namespace
{

// Two independent joints: a wheel (velocity servo) and a pan joint (position servo, limited range).
const char * kModel = R"(
<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="wheel" pos="0 0 0">
      <joint name="wheel_joint" type="hinge" axis="0 0 1" armature="0.01"/>
      <geom size="0.05"/>
    </body>
    <body name="arm" pos="1 0 0">
      <joint name="pan_joint" type="hinge" axis="0 0 1" range="-1.5 1.5" armature="0.01"/>
      <geom size="0.05"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="wheel_joint" joint="wheel_joint" kv="5"/>
    <position name="pan_joint" joint="pan_joint" kp="50" kv="5" ctrlrange="-1.5 1.5"/>
  </actuator>
</mujoco>)";

class EmergencyStopTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char error[512] = "";
    spec_ = mj_parseXMLString(kModel, nullptr, error, sizeof(error));
    ASSERT_NE(spec_, nullptr) << error;
    model_ = mj_compile(spec_, nullptr);
    ASSERT_NE(model_, nullptr);
    data_ = mj_makeData(model_);
  }
  void TearDown() override
  {
    mj_deleteData(data_);
    mj_deleteModel(model_);
    mj_deleteSpec(spec_);
  }
  void run(int steps, double wheel_command, double pan_command)
  {
    for (int i = 0; i < steps; ++i) {
      data_->ctrl[0] = wheel_command;  // what a controller writes every cycle
      data_->ctrl[1] = pan_command;
      stop_.apply(model_, data_);
      mj_step(model_, data_);
    }
  }
  double wheel_speed() const {return data_->qvel[model_->jnt_dofadr[0]];}
  double pan() const {return data_->qpos[model_->jnt_qposadr[1]];}

  mjSpec * spec_{nullptr};
  mjModel * model_{nullptr};
  mjData * data_{nullptr};
  mujoco_ros2_plugins::EmergencyStop stop_;
};

TEST_F(EmergencyStopTest, ActuatorTypesAreClassified)
{
  EXPECT_FALSE(mujoco_ros2_plugins::EmergencyStop::holds_position(model_, 0));  // velocity servo
  EXPECT_TRUE(mujoco_ros2_plugins::EmergencyStop::holds_position(model_, 1));   // position servo
}

TEST_F(EmergencyStopTest, InactiveLeavesTheControllersCommandsAlone)
{
  run(500, 2.0, 0.6);
  EXPECT_DOUBLE_EQ(data_->ctrl[0], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], 0.6);
  EXPECT_GT(wheel_speed(), 1.5);
  EXPECT_NEAR(pan(), 0.6, 0.05);
}

TEST_F(EmergencyStopTest, StopsTheWheelsEvenWhileTheControllerKeepsCommanding)
{
  run(500, 2.0, 0.6);
  ASSERT_GT(wheel_speed(), 1.5);
  stop_.set_active(true);
  run(1000, 2.0, 0.6);
  EXPECT_DOUBLE_EQ(data_->ctrl[0], 0.0);
  EXPECT_NEAR(wheel_speed(), 0.0, 1e-3);
}

TEST_F(EmergencyStopTest, HoldsTheLatchedPositionInsteadOfDrivingToZero)
{
  run(500, 0.0, 0.6);
  const double at_stop = pan();
  stop_.set_active(true);
  run(1000, 0.0, -1.0);  // the controller now asks for something else entirely
  EXPECT_NEAR(pan(), at_stop, 0.02);
  EXPECT_NEAR(data_->ctrl[1], at_stop, 0.05);
  EXPECT_GT(std::abs(data_->ctrl[1]), 0.3);  // not zero
}

TEST_F(EmergencyStopTest, ReleaseHandsControlBack)
{
  run(300, 2.0, 0.6);
  stop_.set_active(true);
  run(300, 2.0, 0.6);
  stop_.set_active(false);
  run(500, 2.0, -0.5);
  EXPECT_DOUBLE_EQ(data_->ctrl[0], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], -0.5);
  EXPECT_GT(wheel_speed(), 1.5);
  EXPECT_NEAR(pan(), -0.5, 0.05);
}

TEST_F(EmergencyStopTest, EnablingTwiceKeepsTheFirstLatchedPosition)
{
  run(500, 0.0, 0.6);
  stop_.set_active(true);
  run(10, 0.0, 0.6);
  const double first = data_->ctrl[1];
  stop_.set_active(true);
  run(200, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], first);
}

TEST_F(EmergencyStopTest, ResetRelatchesFromTheNewWorldState)
{
  run(500, 0.0, 0.6);
  stop_.set_active(true);
  run(10, 0.0, 0.6);
  ASSERT_GT(data_->ctrl[1], 0.3);
  mj_resetData(model_, data_);
  stop_.reset_latch();
  run(10, 0.0, 0.6);
  EXPECT_NEAR(data_->ctrl[1], 0.0, 1e-6);
}

TEST_F(EmergencyStopTest, LatchedTargetStaysWithinTheActuatorRange)
{
  data_->qpos[model_->jnt_qposadr[1]] = 1.7;  // measured slightly outside ctrlrange
  stop_.set_active(true);
  stop_.apply(model_, data_);
  EXPECT_DOUBLE_EQ(data_->ctrl[1], 1.5);
}

}  // namespace
