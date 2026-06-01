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

#include <motion_controller_interface/motion_controller_interface.hpp>

#include <chrono>
#include <string>

#include <rclcpp/logging.hpp>

namespace motion_controller_interface {

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Interface configuration — this controller claims no hardware interfaces.
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
MotionControllerInterface::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
MotionControllerInterface::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

CallbackReturn MotionControllerInterface::on_init() {
  try {
    auto_declare<std::string>("position_controller_name", "position_joint_controller");
    auto_declare<std::string>("effort_controller_name",   "effort_joint_controller");
    auto_declare<std::string>("velocity_controller_name", "velocity_joint_controller");
    auto_declare<double>("command_timeout",       0.5);
    auto_declare<double>("switch_overlap_duration", 0.2);
    auto_declare<std::string>("controller_manager_topic", "/controller_manager");
    auto_declare<std::string>("effort_commands_topic",    "/effort_joint_controller/commands");
    auto_declare<std::string>("velocity_commands_topic",  "/velocity_joint_controller/commands");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionControllerInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  pos_ctrl_name_ = get_node()->get_parameter("position_controller_name").as_string();
  eff_ctrl_name_ = get_node()->get_parameter("effort_controller_name").as_string();
  vel_ctrl_name_ = get_node()->get_parameter("velocity_controller_name").as_string();

  const double timeout_s = get_node()->get_parameter("command_timeout").as_double();
  command_timeout_ns_ = static_cast<int64_t>(timeout_s * 1e9);

  const double overlap_s = get_node()->get_parameter("switch_overlap_duration").as_double();
  switch_overlap_ns_ = static_cast<int64_t>(overlap_s * 1e9);

  const std::string cm_topic =
      get_node()->get_parameter("controller_manager_topic").as_string();
  const std::string effort_cmd_topic =
      get_node()->get_parameter("effort_commands_topic").as_string();
  const std::string velocity_cmd_topic =
      get_node()->get_parameter("velocity_commands_topic").as_string();

  // Service client for switching controllers.
  switch_client_ =
      get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
          cm_topic + "/switch_controller");

  // Subscribe to mode commands: 0=position, 1=effort, 2=velocity.
  mode_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      "~/set_mode", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        const int mode = msg->data;
        if (mode < kPositionMode || mode > kVelocityMode) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Unknown mode %d — must be 0 (position), 1 (effort), or 2 (velocity).",
                      mode);
          return;
        }
        requested_mode_.store(mode);
        const char* names[] = {"position", "effort", "velocity"};
        RCLCPP_INFO(get_node()->get_logger(), "Mode change requested: %s.", names[mode]);
      });

  // Monitor effort commands to reset the safety timeout clock.
  effort_cmd_sub_ =
      get_node()->create_subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>(
          effort_cmd_topic, rclcpp::SystemDefaultsQoS(),
          [this](const tiago_pro_joint_controllers_msgs::msg::JointCommand::SharedPtr) {
            last_effort_cmd_ns_.store(get_node()->now().nanoseconds());
          });

  // Monitor velocity commands to reset the safety timeout clock.
  velocity_cmd_sub_ =
      get_node()->create_subscription<tiago_pro_joint_controllers_msgs::msg::JointCommand>(
          velocity_cmd_topic, rclcpp::SystemDefaultsQoS(),
          [this](const tiago_pro_joint_controllers_msgs::msg::JointCommand::SharedPtr) {
            last_velocity_cmd_ns_.store(get_node()->now().nanoseconds());
          });

  // 10 Hz watchdog: applies mode switches and enforces command timeout.
  watchdog_timer_ =
      get_node()->create_wall_timer(100ms, [this]() { watchdog(); });

  RCLCPP_INFO(get_node()->get_logger(),
              "MotionControllerInterface configured. "
              "position='%s' effort='%s' velocity='%s' timeout=%.2f s overlap=%.2f s. "
              "Publish std_msgs/Int32 to ~/set_mode (0=position, 1=effort, 2=velocity).",
              pos_ctrl_name_.c_str(), eff_ctrl_name_.c_str(), vel_ctrl_name_.c_str(),
              timeout_s, overlap_s);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionControllerInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  requested_mode_.store(kPositionMode);
  current_mode_.store(kPositionMode);
  switch_in_progress_.store(false);
  const int64_t now_ns = get_node()->now().nanoseconds();
  last_effort_cmd_ns_.store(now_ns);
  last_velocity_cmd_ns_.store(now_ns);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionControllerInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  watchdog_timer_->cancel();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Control loop — no hardware writes, just returns OK.
// ---------------------------------------------------------------------------

controller_interface::return_type MotionControllerInterface::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

const std::string& MotionControllerInterface::ctrlForMode(int mode) const {
  switch (mode) {
    case kEffortMode:   return eff_ctrl_name_;
    case kVelocityMode: return vel_ctrl_name_;
    default:            return pos_ctrl_name_;
  }
}

void MotionControllerInterface::activateController(const std::string& activate) {
  if (!switch_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "switch_controller service not ready — cannot switch yet.");
    switch_in_progress_.store(false);
    return;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers   = {activate};
  req->deactivate_controllers = {};
  req->strictness =
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  req->activate_asap = false;

  switch_client_->async_send_request(
      req,
      [this, activate](
          rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture f) {
        if (!f.get()->ok) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Phase-1 switch failed: could not activate '%s'.", activate.c_str());
          switch_in_progress_.store(false);
          return;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Phase-1: '%s' activated (overlap window open).",
                    activate.c_str());
        // Phase-2 fires after the overlap window.
      });
}

void MotionControllerInterface::deactivateController(const std::string& deactivate) {
  if (!switch_client_->service_is_ready()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "switch_controller not ready during phase-2 deactivation of '%s'.",
                deactivate.c_str());
    switch_in_progress_.store(false);
    return;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers   = {};
  req->deactivate_controllers = {deactivate};
  req->strictness =
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  req->activate_asap = false;

  switch_client_->async_send_request(
      req,
      [this, deactivate](
          rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture f) {
        if (f.get()->ok) {
          RCLCPP_INFO(get_node()->get_logger(),
                      "Phase-2: '%s' deactivated. Switch complete.", deactivate.c_str());
        } else {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Phase-2 switch failed: could not deactivate '%s'.",
                       deactivate.c_str());
        }
        switch_in_progress_.store(false);
      });
}

void MotionControllerInterface::watchdog() {
  // Block re-entry while a two-phase switch is in progress.
  if (switch_in_progress_.load()) return;

  const int desired = requested_mode_.load();
  const int current = current_mode_.load();

  // Apply a pending mode change via two-phase overlap switch.
  if (desired != current) {
    const char* names[] = {"position", "effort", "velocity"};
    RCLCPP_INFO(get_node()->get_logger(), "Switching: %s → %s (overlap %.0f ms).",
                names[current], names[desired],
                static_cast<double>(switch_overlap_ns_) / 1e6);

    switch_in_progress_.store(true);
    current_mode_.store(desired);  // Mark as switched so timeout clock starts.

    // Reset the timeout clock for the newly activated mode.
    const int64_t now_ns = get_node()->now().nanoseconds();
    if (desired == kEffortMode)   last_effort_cmd_ns_.store(now_ns);
    if (desired == kVelocityMode) last_velocity_cmd_ns_.store(now_ns);

    const std::string incoming = ctrlForMode(desired);
    const std::string outgoing = ctrlForMode(current);

    // Phase 1: activate the incoming controller (outgoing still running = no torque gap).
    activateController(incoming);

    // Phase 2: deactivate the outgoing controller after the overlap window.
    const auto overlap = std::chrono::nanoseconds(switch_overlap_ns_);
    deactivate_timer_ = get_node()->create_wall_timer(
        overlap, [this, outgoing]() {
          deactivate_timer_->cancel();
          deactivate_timer_.reset();
          deactivateController(outgoing);
        });
    return;
  }

  // Safety watchdog: revert to position if active-mode commands stop.
  if (current != kPositionMode && command_timeout_ns_ > 0) {
    const int64_t last_ns = (current == kEffortMode)
                                ? last_effort_cmd_ns_.load()
                                : last_velocity_cmd_ns_.load();
    const int64_t elapsed_ns = get_node()->now().nanoseconds() - last_ns;
    if (elapsed_ns > command_timeout_ns_) {
      const char* names[] = {"position", "effort", "velocity"};
      RCLCPP_WARN(get_node()->get_logger(),
                  "%s command timeout (%.1f s) — reverting to position.",
                  names[current], static_cast<double>(command_timeout_ns_) / 1e9);
      requested_mode_.store(kPositionMode);
    }
  }
}

}  // namespace motion_controller_interface

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(motion_controller_interface::MotionControllerInterface,
                       controller_interface::ControllerInterface)
