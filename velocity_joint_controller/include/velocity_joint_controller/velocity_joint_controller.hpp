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

namespace velocity_joint_controller {

/**
 * Velocity controller that subscribes to ~/commands
 * (tiago_pro_joint_controllers_msgs/JointCommand) and forwards commanded joint
 * velocities to the hardware velocity command interfaces.
 *
 * On activation the command buffer is initialised with zero velocities so
 * the robot holds its current pose until the first command arrives.
 * On deactivation zero velocities are sent to stop the robot safely.
 *
 * Safety features:
 *   - Per-joint velocity limit validation (configurable, TiagoPro defaults).
 *   - First-order IIR low-pass filter applied in update() to smooth commands.
 *     filter_coeff=1.0 means no filtering; 0.0 means hold previous output.
 *     Reconfigurable at runtime via ROS 2 parameter.
 *
 * Parameters:
 *   robot_type       (string,              default "tiago_pro")
 *   arm_prefix       (string,              default "")
 *   filter_coeff     (double, [0,1],       default 1.0 – no filter)
 *   velocity_limits  (double[7], rad/s,    default TiagoPro)
 *   publish_rate     (double, Hz, >0,      default 500.0) – max rate at which
 *                    velocity commands are written to the hardware interfaces.
 *                    Extra cycles beyond this rate are skipped (TriggerRate).
 *                    Note: on_deactivate() always writes zeros immediately,
 *                    bypassing this rate limit for safety.
 *
 * Topic subscribed:
 *   ~/commands  (tiago_pro_joint_controllers_msgs/msg/JointCommand)
 */
class VelocityJointController : public controller_interface::ControllerInterface {
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
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  std::string robot_type_;
  std::string arm_prefix_;

  std::array<double, kNumJoints> velocity_limits_{};

  // IIR low-pass filter coefficient (thread-safe, runtime-reconfigurable).
  realtime_tools::RealtimeBuffer<double> filter_coeff_buffer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // TriggerRate: limits the rate at which commands are written to hardware.
  int64_t write_period_ns_{2000000LL};  // 1e9 / publish_rate, default 500 Hz
  int64_t accumulated_period_ns_{0};

  // Filtered velocity state maintained across update() calls.
  std::array<double, kNumJoints> filtered_commands_{};

  // Thread-safe command buffer shared between subscriber callback and update().
  realtime_tools::RealtimeBuffer<std::array<double, kNumJoints>> commands_buffer_;

  rclcpp::Subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>::SharedPtr
      commands_subscriber_;
};

}  // namespace velocity_joint_controller
