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
    auto_declare<std::string>("gravity_compensation_controller_name",
                              "gravity_compensation_controller");
    auto_declare<int>("default_mode", kGravityCompensationMode);
    auto_declare<double>("command_timeout",       0.5);
    auto_declare<double>("switch_overlap_duration", 0.2);
    auto_declare<std::string>("controller_manager_topic", "/controller_manager");
    auto_declare<std::string>("effort_commands_topic",    "/effort_joint_controller/commands");
    auto_declare<std::string>("velocity_commands_topic",  "/velocity_joint_controller/commands");
    auto_declare<std::string>("pal_arm_controller_name",  "");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionControllerInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  pos_ctrl_name_  = get_node()->get_parameter("position_controller_name").as_string();
  eff_ctrl_name_  = get_node()->get_parameter("effort_controller_name").as_string();
  vel_ctrl_name_  = get_node()->get_parameter("velocity_controller_name").as_string();
  grav_ctrl_name_ =
      get_node()->get_parameter("gravity_compensation_controller_name").as_string();

  default_mode_ = static_cast<int>(get_node()->get_parameter("default_mode").as_int());
  if (default_mode_ < kPositionMode || default_mode_ > kGravityCompensationMode) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Invalid default_mode %d — must be 0 (position), 1 (effort), 2 (velocity), "
                "or 3 (gravity_compensation). Falling back to position.",
                default_mode_);
    default_mode_ = kPositionMode;
  }

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

  pal_arm_ctrl_name_ = get_node()->get_parameter("pal_arm_controller_name").as_string();

  // Service client for switching controllers.
  switch_client_ =
      get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
          cm_topic + "/switch_controller");

  // Subscribe to mode commands: 0=position, 1=effort, 2=velocity, 3=gravity_compensation.
  mode_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      "~/set_mode", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        const int mode = msg->data;
        if (mode < kPositionMode || mode > kGravityCompensationMode) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Unknown mode %d — must be 0 (position), 1 (effort), "
                      "2 (velocity), or 3 (gravity_compensation).",
                      mode);
          return;
        }
        requested_mode_.store(mode);
        const char* names[] = {"position", "effort", "velocity", "gravity_compensation"};
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
              "position='%s' effort='%s' velocity='%s' gravity_compensation='%s' "
              "pal_arm_controller='%s' default_mode=%d timeout=%.2f s overlap=%.2f s. "
              "Publish std_msgs/Int32 to ~/set_mode "
              "(0=position, 1=effort, 2=velocity, 3=gravity_compensation).",
              pos_ctrl_name_.c_str(), eff_ctrl_name_.c_str(), vel_ctrl_name_.c_str(),
              grav_ctrl_name_.c_str(),
              pal_arm_ctrl_name_.empty() ? "(none)" : pal_arm_ctrl_name_.c_str(),
              default_mode_, timeout_s, overlap_s);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionControllerInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  requested_mode_.store(default_mode_);
  current_mode_.store(default_mode_);
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
    case kEffortMode:              return eff_ctrl_name_;
    case kVelocityMode:            return vel_ctrl_name_;
    case kGravityCompensationMode: return grav_ctrl_name_;
    default:                       return pos_ctrl_name_;
  }
}

void MotionControllerInterface::switchController(const std::string& activate,
                                                  const std::string& deactivate) {
  // PAL's GazeboSystem::perform_command_mode_switch() zeroes the entire
  // joint_control_method_ word when processing stop_interfaces (instead of
  // only clearing that interface's bit).  A two-phase switch — activate first,
  // deactivate second — therefore erases the effort/velocity bit set in phase 1
  // when phase 2 runs.  Sending both in a single SwitchController call causes
  // perform_command_mode_switch to be invoked once: it writes 0 (stop), then
  // writes the correct mode bit (start), leaving the right value in place.
  if (!switch_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "switch_controller service not ready — cannot switch yet.");
    switch_in_progress_.store(false);
    return;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers   = {activate};
  req->deactivate_controllers = {deactivate};

  // PAL's own arm_{side}_controller claims the same `position` command interface and
  // is active by default alongside this coordinator (see pal_arm_controller_name doc).
  // It only conflicts with position mode, so only fold it into the deactivate list here.
  if (!pal_arm_ctrl_name_.empty() && activate == pos_ctrl_name_) {
    req->deactivate_controllers.push_back(pal_arm_ctrl_name_);
  }

  req->strictness =
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  req->activate_asap = false;

  switch_client_->async_send_request(
      req,
      [this, activate, deactivate](
          rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture f) {
        if (f.get()->ok) {
          RCLCPP_INFO(get_node()->get_logger(),
                      "Switch complete: '%s' → '%s'.",
                      deactivate.c_str(), activate.c_str());
        } else {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Switch failed: '%s' → '%s'.",
                       deactivate.c_str(), activate.c_str());
        }
        switch_in_progress_.store(false);
      });
}

void MotionControllerInterface::watchdog() {
  // Block re-entry while a switch is in progress.
  if (switch_in_progress_.load()) return;

  const int desired = requested_mode_.load();
  const int current = current_mode_.load();

  // Apply a pending mode change via a single atomic switch_controller call.
  if (desired != current) {
    const char* names[] = {"position", "effort", "velocity", "gravity_compensation"};
    RCLCPP_INFO(get_node()->get_logger(), "Switching: %s → %s.",
                names[current], names[desired]);

    switch_in_progress_.store(true);
    current_mode_.store(desired);  // Mark as switched so timeout clock starts.

    // Reset the timeout clock for the newly activated mode.
    const int64_t now_ns = get_node()->now().nanoseconds();
    if (desired == kEffortMode)   last_effort_cmd_ns_.store(now_ns);
    if (desired == kVelocityMode) last_velocity_cmd_ns_.store(now_ns);

    switchController(ctrlForMode(desired), ctrlForMode(current));
    return;
  }

  // Safety watchdog: revert to default if active-mode commands stop.
  // Only applies to effort and velocity modes — gravity_compensation has no
  // commands topic and stays active until a different mode is requested.
  if ((current == kEffortMode || current == kVelocityMode) && command_timeout_ns_ > 0) {
    const int64_t last_ns = (current == kEffortMode)
                                ? last_effort_cmd_ns_.load()
                                : last_velocity_cmd_ns_.load();
    const int64_t elapsed_ns = get_node()->now().nanoseconds() - last_ns;
    if (elapsed_ns > command_timeout_ns_) {
      const char* names[] = {"position", "effort", "velocity", "gravity_compensation"};
      RCLCPP_WARN(get_node()->get_logger(),
                  "%s command timeout (%.1f s) — reverting to gravity_compensation.",
                  names[current], static_cast<double>(command_timeout_ns_) / 1e9);
      requested_mode_.store(kGravityCompensationMode);
    }
  }
}

}  // namespace motion_controller_interface

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(motion_controller_interface::MotionControllerInterface,
                       controller_interface::ControllerInterface)
