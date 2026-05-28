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

#include <effort_joint_controller/effort_joint_controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>

#include <rclcpp/logging.hpp>

namespace effort_joint_controller {

// TiagoPro per-joint torque limits in N*m
static constexpr std::array<double, 7> kDefaultEffortLimits{87.0, 87.0, 87.0, 87.0,
                                                              12.0, 12.0, 12.0};

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
EffortJointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ +  std::to_string(i) + "_joint"  + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
EffortJointController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

CallbackReturn EffortJointController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "tiago_pro");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<double>>(
        "effort_limits",
        std::vector<double>(kDefaultEffortLimits.begin(), kDefaultEffortLimits.end()));
    auto_declare<double>("delta_tau_max", 1.0);
    auto_declare<double>("publish_rate", 500.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during on_init with message: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn EffortJointController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  delta_tau_max_ = get_node()->get_parameter("delta_tau_max").as_double();

  const double publish_rate = get_node()->get_parameter("publish_rate").as_double();
  if (publish_rate <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "publish_rate must be positive, got %.1f.", publish_rate);
    return CallbackReturn::ERROR;
  }
  write_period_ns_ = static_cast<int64_t>(1e9 / publish_rate);

  const auto limits_vec = get_node()->get_parameter("effort_limits").as_double_array();
  if (static_cast<int>(limits_vec.size()) != kNumJoints) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "effort_limits must have exactly %d entries, got %zu.", kNumJoints,
                 limits_vec.size());
    return CallbackReturn::ERROR;
  }
  std::copy(limits_vec.begin(), limits_vec.end(), effort_limits_.begin());

  // Build expected joint name list for message validation.
  std::vector<std::string> expected_names;
  for (int i = 1; i <= kNumJoints; ++i) {
    expected_names.push_back(arm_prefix_ + std::to_string(i) + "_joint" );
  }

  commands_subscriber_ =
      get_node()->create_subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>(
          "~/commands", rclcpp::SystemDefaultsQoS(),
          [this, expected_names](
              const tiago_pro_joint_controllers_msgs::msg::JointCommand::SharedPtr msg) {
            // Validate joint_names when provided.
            if (!msg->joint_names.empty()) {
              if (msg->joint_names.size() != static_cast<size_t>(kNumJoints) ||
                  msg->joint_names != expected_names) {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "JointCommand has unexpected joint_names – dropping message.");
                return;
              }
            }
            if (static_cast<int>(msg->command.size()) != kNumJoints) {
              RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                    "JointCommand.command must have %d entries, got %zu.",
                                    kNumJoints, msg->command.size());
              return;
            }
            // Per-joint torque limit check.
            for (int i = 0; i < kNumJoints; ++i) {
              if (std::abs(msg->command[i]) > effort_limits_[i]) {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "Commanded torque for joint %d (%.2f N*m) exceeds limit (%.2f N*m) – "
                    "dropping message.",
                    i + 1, msg->command[i], effort_limits_[i]);
                return;
              }
            }
            std::array<double, kNumJoints> commands{};
            std::copy(msg->command.begin(), msg->command.end(), commands.begin());
            commands_buffer_.writeFromNonRT(commands);
          });

  RCLCPP_INFO(get_node()->get_logger(),
              "EffortJointController configured for robot '%s%s'. delta_tau_max=%.2f N*m/cycle, "
              "publish_rate=%.1f Hz.",
              arm_prefix_.c_str(), robot_type_.c_str(), delta_tau_max_, publish_rate);
  return CallbackReturn::SUCCESS;
}

CallbackReturn EffortJointController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::array<double, kNumJoints> zeros{};
  zeros.fill(0.0);
  commands_buffer_.writeFromNonRT(zeros);
  prev_commands_ = zeros;
  accumulated_period_ns_ = 0;
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

controller_interface::return_type EffortJointController::update(const rclcpp::Time& /*time*/,
                                                                const rclcpp::Duration& period) {
  accumulated_period_ns_ += period.nanoseconds();
  if (accumulated_period_ns_ < write_period_ns_) {
    return controller_interface::return_type::OK;
  }
  accumulated_period_ns_ = 0;

  const auto target = *commands_buffer_.readFromRT();
  const auto saturated = saturateTorqueRate(target, prev_commands_);

  for (int i = 0; i < kNumJoints; ++i) {
    if (!command_interfaces_[i].set_value(saturated[i])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set effort command on interface '%s'.",
                   command_interfaces_[i].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
  }
  prev_commands_ = saturated;
  return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::array<double, EffortJointController::kNumJoints> EffortJointController::saturateTorqueRate(
    const std::array<double, kNumJoints>& target,
    const std::array<double, kNumJoints>& prev) const {
  std::array<double, kNumJoints> out{};
  for (int i = 0; i < kNumJoints; ++i) {
    const double diff = target[i] - prev[i];
    out[i] = prev[i] + std::max(std::min(diff, delta_tau_max_), -delta_tau_max_);
  }
  return out;
}

}  // namespace effort_joint_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(effort_joint_controller::EffortJointController,
                       controller_interface::ControllerInterface)
