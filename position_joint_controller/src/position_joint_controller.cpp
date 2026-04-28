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

#include <position_joint_controller/position_joint_controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>

#include <rclcpp/logging.hpp>

namespace position_joint_controller {

// TiagoPro per-joint position limits in rad
static constexpr std::array<double, 7> kDefaultPositionLimitsLower{
    -4.7123, -2.4434, -2.6179, -2.4434, -1.5708, -1.8849, -2.6179};
static constexpr std::array<double, 7> kDefaultPositionLimitsUpper{
    0.5235, 1.1344, 2.6179, 1.1344, 3.6651, 3.0019, 2.6179};

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
PositionJointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ + std::to_string(i) + "_joint" +
                           "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
PositionJointController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ + std::to_string(i) + "_joint" +
                           "/position");
  }
  return config;
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

CallbackReturn PositionJointController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "tiago_pro");
    auto_declare<std::string>("arm_prefix", "left_arm");
    auto_declare<double>("filter_coeff", 0.3);
    auto_declare<std::vector<double>>(
        "position_limits_lower",
        std::vector<double>(kDefaultPositionLimitsLower.begin(), kDefaultPositionLimitsLower.end()));
    auto_declare<std::vector<double>>(
        "position_limits_upper",
        std::vector<double>(kDefaultPositionLimitsUpper.begin(), kDefaultPositionLimitsUpper.end()));
    auto_declare<int>("publish_rate", 100);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during on_init with message: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PositionJointController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

  const double filter_coeff = get_node()->get_parameter("filter_coeff").as_double();
  if (filter_coeff < 0.0 || filter_coeff > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "filter_coeff must be in [0, 1], got %.3f.", filter_coeff);
    return CallbackReturn::ERROR;
  }
  filter_coeff_buffer_.writeFromNonRT(filter_coeff);

  const auto lower_vec = get_node()->get_parameter("position_limits_lower").as_double_array();
  const auto upper_vec = get_node()->get_parameter("position_limits_upper").as_double_array();
  if (static_cast<int>(lower_vec.size()) != kNumJoints ||
      static_cast<int>(upper_vec.size()) != kNumJoints) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "position_limits_lower and position_limits_upper must each have %d entries.",
                 kNumJoints);
    return CallbackReturn::ERROR;
  }
  std::copy(lower_vec.begin(), lower_vec.end(), position_limits_lower_.begin());
  std::copy(upper_vec.begin(), upper_vec.end(), position_limits_upper_.begin());

  const int publish_rate = get_node()->get_parameter("publish_rate").as_int();
  if (publish_rate <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "publish_rate must be positive, got %d.", publish_rate);
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
            // Per-joint position limit check.
            for (int i = 0; i < kNumJoints; ++i) {
              if (msg->command[i] < position_limits_lower_[i] ||
                  msg->command[i] > position_limits_upper_[i]) {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "Commanded position for joint %d (%.4f rad) outside limits [%.4f, %.4f] – "
                    "dropping message.",
                    i + 1, msg->command[i], position_limits_lower_[i], position_limits_upper_[i]);
                return;
              }
            }
            std::array<double, kNumJoints> commands{};
            std::copy(msg->command.begin(), msg->command.end(), commands.begin());
            commands_buffer_.writeFromNonRT(commands);
          });

  RCLCPP_INFO(get_node()->get_logger(),
              "PositionJointController configured for robot '%s%s'. filter_coeff=%.2f, "
              "publish_rate=%d Hz.",
              arm_prefix_.c_str(), robot_type_.c_str(), filter_coeff, publish_rate);
  return CallbackReturn::SUCCESS;
}

CallbackReturn PositionJointController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Seed the command buffer with the current joint positions so the robot
  // holds its pose until the first command arrives via the topic.
  std::array<double, kNumJoints> current_positions{};
  for (int i = 0; i < kNumJoints; ++i) {
    const auto value = state_interfaces_[i].get_optional();
    if (!value.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to read position from state interface '%s' on activation.",
                   state_interfaces_[i].get_name().c_str());
      return CallbackReturn::ERROR;
    }
    current_positions[i] = value.value();
  }
  commands_buffer_.writeFromNonRT(current_positions);
  filtered_commands_ = current_positions;
  accumulated_period_ns_ = 0;

  RCLCPP_INFO(get_node()->get_logger(),
              "PositionJointController activated. Holding current joint positions.");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

controller_interface::return_type PositionJointController::update(
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
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set position command on interface '%s'.",
                   command_interfaces_[i].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace position_joint_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(position_joint_controller::PositionJointController,
                       controller_interface::ControllerInterface)
