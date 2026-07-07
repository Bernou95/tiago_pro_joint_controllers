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

#include <atomic>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tiago_pro_joint_controllers_msgs/msg/joint_command.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace motion_controller_interface {

/**
 * Coordinator ros2_control plugin that handles runtime switching between a
 * position controller, a velocity controller, an effort controller, and a
 * gravity-compensation controller.
 *
 * Claims no hardware interfaces. All real work runs in non-RT callbacks:
 *   - 10 Hz watchdog timer: applies pending mode switches and checks command timeout.
 *   - ~/set_mode subscription (std_msgs/Int32):
 *       0 = position mode
 *       1 = effort mode
 *       2 = velocity mode
 *       3 = gravity-compensation mode
 *   - Effort / velocity command monitor subscriptions: reset the timeout clock.
 *
 * The safety timeout (command_timeout) applies to effort and velocity modes only.
 * If no command is received on the active mode's topic within the timeout, the
 * controller reverts to position mode automatically. Gravity-compensation mode
 * has no commands topic and so is not timed out — it stays active until the
 * user requests a different mode.
 *
 * Parameters:
 *   position_controller_name             (string,  default "position_joint_controller")
 *   effort_controller_name               (string,  default "effort_joint_controller")
 *   velocity_controller_name             (string,  default "velocity_joint_controller")
 *   gravity_compensation_controller_name (string,  default "gravity_compensation_controller")
 *   default_mode                         (int,     default 0 — the mode assumed active on
 *                                         activation: 0=position, 1=effort, 2=velocity,
 *                                         3=gravity_compensation. Must match whichever of the
 *                                         four controllers was actually spawned active, or the
 *                                         first requested switch will compute the wrong
 *                                         deactivate target.)
 *   command_timeout                      (double,  default 0.5 s — 0 disables timeout)
 *   controller_manager_topic             (string,  default "/controller_manager")
 *   effort_commands_topic                (string,  default "/effort_joint_controller/commands")
 *   velocity_commands_topic              (string,  default "/velocity_joint_controller/commands")
 *   pal_arm_controller_name              (string,  default "" — disabled. On the real robot,
 *                                         PAL's own arm_{side}_controller (a
 *                                         JointTrajectoryController) claims the same `position`
 *                                         command interface as position_controller_name and is
 *                                         active by default alongside this coordinator. If set,
 *                                         it is added to the deactivate list whenever switching
 *                                         TO position mode (and only then — effort, velocity, and
 *                                         gravity_compensation modes don't conflict with it),
 *                                         so the claim is released atomically in the same
 *                                         switch_controller call instead of being left active.)
 */
class MotionControllerInterface : public controller_interface::ControllerInterface {
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
  static constexpr int kPositionMode             = 0;
  static constexpr int kEffortMode               = 1;
  static constexpr int kVelocityMode             = 2;
  static constexpr int kGravityCompensationMode  = 3;

  std::string pos_ctrl_name_{"position_joint_controller"};
  std::string eff_ctrl_name_{"effort_joint_controller"};
  std::string vel_ctrl_name_{"velocity_joint_controller"};
  std::string grav_ctrl_name_{"gravity_compensation_controller"};
  // PAL's own arm_{side}_controller name, if any — see pal_arm_controller_name doc above.
  std::string pal_arm_ctrl_name_{""};
  // Mode assumed active on activation; read from the "default_mode" parameter.
  int default_mode_{kPositionMode};
  int64_t command_timeout_ns_{500000000LL};    // 0.5 s
  // How long the incoming and outgoing controllers overlap during a switch.
  // During this window both position (holding) and effort/velocity (ramping up)
  // are simultaneously active, preventing any zero-torque gap.
  int64_t switch_overlap_ns_{200000000LL};     // 0.2 s

  // Shared between RT update() and non-RT callbacks via atomics.
  std::atomic<int>     requested_mode_{kPositionMode};
  std::atomic<int>     current_mode_{kPositionMode};
  std::atomic<bool>    switch_in_progress_{false};
  std::atomic<int64_t> last_effort_cmd_ns_{0};
  std::atomic<int64_t> last_velocity_cmd_ns_{0};

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>::SharedPtr
      effort_cmd_sub_;
  rclcpp::Subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>::SharedPtr
      velocity_cmd_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Returns the controller name that corresponds to the given mode.
  [[nodiscard]] const std::string& ctrlForMode(int mode) const;

  // Activate incoming and deactivate outgoing in one SwitchController call.
  // A single call is required because PAL's GazeboSystem::perform_command_mode_switch
  // zeroes the entire joint_control_method_ word on stop (not just the relevant bit),
  // so a two-phase switch erases the mode bit set in phase 1 when phase 2 runs.
  void switchController(const std::string& activate, const std::string& deactivate);

  // Watchdog callback (10 Hz): applies pending mode switches and enforces timeout.
  void watchdog();
};

}  // namespace motion_controller_interface
