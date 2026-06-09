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

#include <velocity_joint_controller/velocity_joint_controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>

#include <rclcpp/logging.hpp>

namespace velocity_joint_controller {

// TiagoPro per-joint velocity limits in rad/s
static constexpr std::array<double, 7> kDefaultVelocityLimits{2.175, 2.175, 2.175, 2.175,
                                                               2.61,  2.61,  2.61};

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
VelocityJointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ + std::to_string(i) + "_joint/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
VelocityJointController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

CallbackReturn VelocityJointController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "tiago_pro");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<double>("filter_coeff", 1.0);
    auto_declare<std::vector<double>>(
        "velocity_limits",
        std::vector<double>(kDefaultVelocityLimits.begin(), kDefaultVelocityLimits.end()));
    auto_declare<double>("publish_rate", 500.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during on_init with message: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn VelocityJointController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

  const double filter_coeff = get_node()->get_parameter("filter_coeff").as_double();
  if (filter_coeff < 0.0 || filter_coeff > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "filter_coeff must be in [0, 1], got %.3f.",
                 filter_coeff);
    return CallbackReturn::ERROR;
  }
  filter_coeff_buffer_.writeFromNonRT(filter_coeff);

  const auto limits_vec = get_node()->get_parameter("velocity_limits").as_double_array();
  if (static_cast<int>(limits_vec.size()) != kNumJoints) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "velocity_limits must have exactly %d entries, got %zu.", kNumJoints,
                 limits_vec.size());
    return CallbackReturn::ERROR;
  }
  std::copy(limits_vec.begin(), limits_vec.end(), velocity_limits_.begin());

  const double publish_rate = get_node()->get_parameter("publish_rate").as_double();
  if (publish_rate <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "publish_rate must be positive, got %.1f.", publish_rate);
    return CallbackReturn::ERROR;
  }
  write_period_ns_ = static_cast<int64_t>(1e9 / publish_rate);

  // Register runtime parameter callback for filter_coeff.
  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
          if (param.get_name() == "filter_coeff") {
            const double val = param.as_double();
            if (val < 0.0 || val > 1.0) {
              result.successful = false;
              result.reason = "filter_coeff must be in [0, 1].";
              return result;
            }
            filter_coeff_buffer_.writeFromNonRT(val);
            RCLCPP_INFO(get_node()->get_logger(), "filter_coeff updated to %.3f.", val);
          }
        }
        return result;
      });

  // Build expected joint name list for message validation.
  std::vector<std::string> expected_names;
  for (int i = 1; i <= kNumJoints; ++i) {
    expected_names.push_back(arm_prefix_ + std::to_string(i) + "_joint");
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
            // Per-joint velocity limit check.
            for (int i = 0; i < kNumJoints; ++i) {
              if (std::abs(msg->command[i]) > velocity_limits_[i]) {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "Commanded velocity for joint %d (%.3f rad/s) exceeds limit (%.3f rad/s) – "
                    "dropping message.",
                    i + 1, msg->command[i], velocity_limits_[i]);
                return;
              }
            }
            std::array<double, kNumJoints> commands{};
            std::copy(msg->command.begin(), msg->command.end(), commands.begin());
            commands_buffer_.writeFromNonRT(commands);
          });

  RCLCPP_INFO(get_node()->get_logger(),
              "VelocityJointController configured for robot '%s%s'. filter_coeff=%.2f, "
              "publish_rate=%.1f Hz.",
              arm_prefix_.c_str(), robot_type_.c_str(), filter_coeff, publish_rate);
  return CallbackReturn::SUCCESS;
}

CallbackReturn VelocityJointController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::array<double, kNumJoints> zeros{};
  zeros.fill(0.0);
  commands_buffer_.writeFromNonRT(zeros);
  filtered_commands_ = zeros;
  accumulated_period_ns_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn VelocityJointController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Send zero velocities on deactivation to stop the robot safely.
  std::array<double, kNumJoints> zeros{};
  zeros.fill(0.0);
  for (int i = 0; i < kNumJoints; ++i) {
    if (!command_interfaces_[i].set_value(zeros[i])) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Failed to set zero velocity on interface '%s' during deactivation.",
                  command_interfaces_[i].get_name().c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

controller_interface::return_type VelocityJointController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  accumulated_period_ns_ += period.nanoseconds();
  if (accumulated_period_ns_ < write_period_ns_) {
    return controller_interface::return_type::OK;
  }
  accumulated_period_ns_ = 0;

  const auto target = *commands_buffer_.readFromRT();
  const double alpha = *filter_coeff_buffer_.readFromRT();

  for (int i = 0; i < kNumJoints; ++i) {
    filtered_commands_[i] = alpha * target[i] + (1.0 - alpha) * filtered_commands_[i];
    if (!command_interfaces_[i].set_value(filtered_commands_[i])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set velocity command on interface '%s'.",
                   command_interfaces_[i].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace velocity_joint_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(velocity_joint_controller::VelocityJointController,
                       controller_interface::ControllerInterface)
