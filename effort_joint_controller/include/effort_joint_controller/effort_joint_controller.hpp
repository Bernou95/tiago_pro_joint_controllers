// Copyright (c) 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <tiago_pro_joint_controllers_msgs/msg/joint_command.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace effort_joint_controller {

/**
 * Effort (torque) controller that subscribes to ~/commands
 * (tiago_pro_joint_controllers_msgs/JointCommand) and forwards commanded torques
 * to the hardware effort interfaces.
 *
 * Safety features:
 *   - Per-joint torque limit validation (configurable, TiagoPro defaults).
 *   - Torque-rate saturation: clamps per-cycle torque change to delta_tau_max
 *     to prevent hardware safety stops from triggering.
 *   - Falls back to zero torques (gravity compensation) when no command has
 *     been received or when a command is rejected.
 *
 * Parameters:
 *   robot_type     (string,             default "tiago_pro")
 *   arm_prefix     (string,             default "")
 *   effort_limits  (double[7], N*m,     default TiagoPro: [87,87,87,87,12,12,12])
 *   delta_tau_max  (double,   N*m/cycle, default 1.0)
 *   publish_rate   (double, Hz, >0,     default 500.0) – max rate at which
 *                  torque commands are written to the hardware interfaces.
 *                  Extra cycles beyond this rate are skipped (TriggerRate).
 *
 * Topic subscribed:
 *   ~/commands  (tiago_pro_joint_controllers_msgs/msg/JointCommand)
 */
class EffortJointController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  std::string robot_type_;
  std::string arm_prefix_;

  std::array<double, kNumJoints> effort_limits_{};
  double delta_tau_max_{1.0};

  // TriggerRate: limits the rate at which torque commands are written to hardware.
  int64_t write_period_ns_{2000000LL};  // 1e9 / publish_rate, default 500 Hz
  int64_t accumulated_period_ns_{0};

  // Previous commanded torques — needed for rate saturation in update().
  std::array<double, kNumJoints> prev_commands_{};

  // Thread-safe buffer shared between the subscriber callback and update().
  realtime_tools::RealtimeBuffer<std::array<double, kNumJoints>> commands_buffer_;

  rclcpp::Subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>::SharedPtr
      commands_subscriber_;

  // Clamps the torque derivative to ±delta_tau_max per control cycle.
  [[nodiscard]] std::array<double, kNumJoints> saturateTorqueRate(
      const std::array<double, kNumJoints>& target,
      const std::array<double, kNumJoints>& prev) const;
};

}  // namespace effort_joint_controller
