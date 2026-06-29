# tiago_pro_joint_controllers

ROS 2 (Humble) `ros2_control` controller plugins for PAL Robotics' **TiagoPro** arm(s): topic-driven position, effort (torque), and velocity control, plus a coordinator that switches between them at runtime with an automatic safety fallback.

## 1. Project Overview

TiagoPro's stock `ros2_control` setup activates one controller at a time and expects reconfiguration through the `controller_manager` for every mode change. That's fine for a fixed deployment, but it's awkward for research and application code that wants to:

- send simple, uniform joint commands over a ROS 2 topic (no action servers, no trajectory messages),
- switch between position / effort / velocity control **at runtime** (e.g. hold a pose, then apply torques, then go back to holding), and
- get a safety net for free — if effort/velocity commands stop arriving, the arm should not be left coasting or holding a stale torque.

This package provides that layer:

- **`position_joint_controller`**, **`effort_joint_controller`**, **`velocity_joint_controller`** — independent `ControllerInterface` plugins, each taking commands on its own `~/commands` topic, each enforcing TiagoPro's joint limits and applying smoothing/rate-limiting before writing to hardware.
- **`motion_controller_interface`** — a coordinator plugin with no hardware interfaces of its own. It listens for a desired mode on `~/set_mode`, calls `controller_manager`'s `switch_controller` service to activate/deactivate the right controller, and reverts to position mode automatically if effort/velocity commands stop arriving (configurable timeout).
- **`tiago_pro_joint_controllers_msgs`** — the single message type (`JointCommand`) shared by all three command topics.
- **`tiago_pro_joint_controllers`** — meta-package that pulls all of the above together as one dependency.

Everything here is built against TiagoPro's 7-DOF arm joint naming (`{arm_prefix}1_joint` … `{arm_prefix}7_joint`) and is meant to run alongside PAL's own bringup stack (`tiago_pro_bringup`), either on the real robot or in Gazebo Classic.

## 2. Data Interface

### Input (what the system receives)

| Topic | Type | Consumed by | Meaning |
|---|---|---|---|
| `<controller_name>/commands` | `tiago_pro_joint_controllers_msgs/msg/JointCommand` | `effort_joint_controller`, `position_joint_controller`, `velocity_joint_controller` | One value per joint: **N·m** for effort, **rad** for position, **rad/s** for velocity. |
| `<gravity_compensation_topic>` (optional, configurable name) | `tiago_pro_joint_controllers_msgs/msg/JointCommand` | `effort_joint_controller` | Per-joint gravity feedforward torques (N·m), added to the commanded effort before rate-limiting. Only active if `use_gravity_compensation: true`. |
| `<motion_controller_interface>/set_mode` | `std_msgs/msg/Int32` | `motion_controller_interface` | Desired mode: `0`=position, `1`=effort, `2`=velocity, `3`=gravity_compensation. |
| `<effort_commands_topic>` / `<velocity_commands_topic>` (same topics as above, monitored read-only) | `tiago_pro_joint_controllers_msgs/msg/JointCommand` | `motion_controller_interface` | Used only to reset the command-timeout clock — not consumed for control. |

`JointCommand` (defined in `tiago_pro_joint_controllers_msgs/msg/JointCommand.msg`):

```
std_msgs/Header header
string[] joint_names   # optional; if empty, standard TiagoPro joint order + arm_prefix is assumed
float64[] command       # one value per joint (7 entries for a TiagoPro arm)
```

Every controller validates incoming messages against its configured per-joint limits and drops (with a throttled log) any message that is malformed or out of range — out-of-range commands never reach the hardware.

### Output (what the system generates)

- **Hardware command interfaces**: each controller writes directly into the `ros2_control` command interfaces it claims — `{arm_prefix}N_joint/effort`, `{arm_prefix}N_joint/position`, or `{arm_prefix}N_joint/velocity` (N = 1..7). There is no separate "output topic"; the hardware interface *is* the output, consumed by whatever `hardware_interface::SystemInterface` is loaded (real TiagoPro driver or Gazebo).
- **Controller switching**: `motion_controller_interface` calls the `/controller_manager/switch_controller` service to activate one controller and deactivate another in a single request (see [Software Architecture](#4-software-architecture) for why it must be a single call).
- **Logging**: all nodes log configuration on startup and throttled warnings/errors on rejected commands or failed interface writes via `RCLCPP_*` macros — no other telemetry topics are published.

## 3. Directory & File Structure

```
tiago_pro_joint_controllers/            # this repo
├── tiago_pro_joint_controllers/        # meta-package: depends on every package below, no code
├── tiago_pro_joint_controllers_msgs/   # the shared JointCommand.msg interface package
│   └── msg/JointCommand.msg
├── effort_joint_controller/            # torque control plugin
│   ├── include/.../effort_joint_controller.hpp   # class + parameter docs
│   ├── src/effort_joint_controller.cpp           # implementation
│   ├── effort_joint_controller.xml               # pluginlib export descriptor
│   ├── config/*.yaml                             # per-arm / real-robot / Gazebo parameter sets
│   └── launch/*.launch.py                        # real-robot and Gazebo Classic launchers
├── position_joint_controller/          # position control plugin (same layout as above)
├── velocity_joint_controller/          # velocity control plugin (same layout as above)
└── motion_controller_interface/        # runtime mode-switch coordinator
    ├── include/.../motion_controller_interface.hpp
    ├── src/motion_controller_interface.cpp
    ├── motion_controller_interface.xml
    └── config/motion_controller_gazebo.yaml
```

Notes on the repeated per-controller layout (`effort_joint_controller`, `position_joint_controller`, `velocity_joint_controller`):

- **`include/<pkg>/<pkg>.hpp`** — the `ControllerInterface` subclass declaration; this is also where parameters and topics are documented in docstrings.
- **`src/<pkg>.cpp`** — lifecycle callbacks (`on_init`, `on_configure`, `on_activate`[, `on_deactivate`]), the `update()` control loop, and the `PLUGINLIB_EXPORT_CLASS` registration.
- **`<pkg>.xml`** — pluginlib plugin description, referenced from each `CMakeLists.txt` via `pluginlib_export_plugin_description_file()` and loaded by `controller_manager` at runtime.
- **`config/`** — one YAML per scenario: `left_arm_controllers.yaml` / `right_arm_controllers.yaml` (real robot), `left_arm_gazebo.yaml` / `right_arm_gazebo.yaml` / `dual_arm_gazebo.yaml` (simulation), plus multi-controller variants for `effort_joint_controller` used when running position+effort+velocity together in Gazebo. See `9jun.md` for the naming-convention rationale.
- **`launch/`** — Python launch files only (XML real-robot launchers were retired, see `9jun.md`). Each accepts an `arm_side` argument (`left`/`right`/`both`) that selects the matching config YAML via an `OpaqueFunction`.

`9jun.md` is a changelog describing the arm-agnostic multi-arm refactor; useful background but not required reading to use the package.

## 4. Software Architecture

All four controllers derive from the same `ros2_control` base class and are loaded as `pluginlib` plugins by `controller_manager`:

```
                         controller_manager (ros2_control)
                                   │ loads via pluginlib
   ┌───────────────────────────────┼───────────────────────────────┬───────────────────────────────┐
   │                               │                               │                               │
   ▼                               ▼                               ▼                               ▼
EffortJointController      PositionJointController      VelocityJointController      MotionControllerInterface
(controller_interface::ControllerInterface)                                          (claims NO hw interfaces)

~/commands (JointCommand)   ~/commands (JointCommand)   ~/commands (JointCommand)     ~/set_mode (Int32)
       │ limit-check               │ limit-check               │ limit-check                 │
       ▼                           ▼                           ▼                             ▼
RealtimeBuffer<torques>    RealtimeBuffer<positions>   RealtimeBuffer<velocities>     requested_mode_ (atomic)
       │                           │                           │                             │
   update():                   update():                   update():               10 Hz watchdog():
   TriggerRate gate           TriggerRate gate           TriggerRate gate           - detect mode change
   + gravity comp.            + IIR low-pass filter      + IIR low-pass filter      - call switch_controller
   + rate-saturate                                       + zero-on-deactivate         (single combined
       │                           │                           │                       activate+deactivate
       ▼                           ▼                           ▼                       request — see note)
 {arm}N_joint/effort        {arm}N_joint/position       {arm}N_joint/velocity        - command-timeout watchdog
   (hardware interface)       (hardware interface)         (hardware interface)        → reverts to position
```

Key architectural points:

- **Shared base, independent controllers.** `EffortJointController`, `PositionJointController`, and `VelocityJointController` each implement `controller_interface::ControllerInterface` and run independently — only one is *active* at a time per arm (`controller_manager` enforces exclusive claims on the same hardware interfaces).
- **RT-safe command path.** Each controller's subscription callback (non-RT) validates a `JointCommand` and writes it into a `realtime_tools::RealtimeBuffer`; the RT `update()` loop reads from that buffer — the standard `ros2_control` pattern for crossing the RT/non-RT boundary without locks.
- **TriggerRate.** `update()` may be called faster than a controller's configured `publish_rate`; each controller accumulates elapsed time and only writes to hardware once enough time has passed, so torque/position/velocity writes happen at a bounded, configurable rate.
- **MotionControllerInterface is hardware-free.** It claims `NONE` for both command and state interfaces — all of its work (mode switching, timeout detection) happens in non-RT callbacks (a 10 Hz wall timer + topic callbacks), not in `update()`.
- **Single-call switching.** PAL's Gazebo `GazeboSystem::perform_command_mode_switch()` zeroes the *entire* `joint_control_method_` word on a stop, not just the relevant bit — so activating and deactivating controllers in two separate `switch_controller` calls would erase the mode bit set by the first call. `motion_controller_interface` works around this by sending both the activate and deactivate controller names in **one** `SwitchController` request.
- **Auto-revert safety.** While in effort or velocity mode, `motion_controller_interface` tracks the time since the last command on that mode's topic; if `command_timeout` (default 0.5 s) elapses with no new command, it requests a switch back to position mode. Gravity-compensation mode has no commands topic and is therefore not subject to this timeout.

## 5. Requirements & Dependencies

### Software

- **ROS 2 Humble** with `ament_cmake` and `colcon`.
- **ros2_control stack**: `controller_interface`, `hardware_interface`, `controller_manager`, `controller_manager_msgs`.
- **Common ROS 2 libs**: `rclcpp`, `rclcpp_lifecycle`, `pluginlib`, `realtime_tools`, `std_msgs`.
- **Message generation**: `rosidl_default_generators` / `rosidl_default_runtime` (for `tiago_pro_joint_controllers_msgs`).
- **PAL-specific**: `launch_pal` and `tiago_pro_bringup` (used by the real-robot launch files to bring up the robot description, hardware interface, and `controller_manager` before spawning these controllers).
- **Simulation (optional)**: Gazebo Classic, for the `*_gazebo_classic.launch.py` / `*_gazebo.launch.py` launch files.

All of the above — including `tiago_pro_bringup` and Gazebo Classic — are already provided in **PAL Robotics' official TiagoPro Docker development image**. The recommended setup is to clone this repo into that image's workspace `src/` rather than assembling the dependency stack manually outside of it.

### Hardware

- A TiagoPro robot with one or both 7-DOF arms, reachable over the network (`robot_ip` launch argument) — **or**
- Gazebo Classic simulation of TiagoPro (no physical robot needed) — **or**
- `use_fake_hardware:=true` for a hardware-less smoke test of the launch/control stack.

## 6. Getting Started Guide

These steps assume you are working inside PAL's TiagoPro Docker image (or an equivalent ROS 2 Humble workspace that already has `tiago_pro_bringup` available).

### Build

```bash
cd ~/humble_ws          # workspace root containing src/tiago_pro_joint_controllers
colcon build --symlink-install --packages-up-to tiago_pro_joint_controllers
source install/setup.bash
```

### Run — Gazebo Classic simulation

```bash
# Dual-arm, all three controllers loaded (mode-switchable)
ros2 launch effort_joint_controller multi_controller_gazebo_classic.launch.py

# Left arm only
ros2 launch effort_joint_controller multi_controller_gazebo_classic.launch.py arm_side:=left

# Single-controller simulation (position only, right arm)
ros2 launch position_joint_controller position_joint_controller_gazebo_classic.launch.py arm_side:=right
```

### Run — real robot

```bash
# Both arms, effort control
ros2 launch effort_joint_controller effort_joint_controller.launch.py robot_ip:=<ROBOT_IP>

# Right arm only, velocity control
ros2 launch velocity_joint_controller velocity_joint_controller.launch.py robot_ip:=<ROBOT_IP> arm_side:=right
```

### Verify it's running

```bash
ros2 control list_controllers
```

### Send a command

```bash
# Example: command the left-arm effort controller (7 joints, units = N*m)
ros2 topic pub --once /arm_left_effort_joint_controller/commands \
  tiago_pro_joint_controllers_msgs/msg/JointCommand \
  "{command: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### Switch modes at runtime (with `motion_controller_interface` loaded)

```bash
# Switch left arm to effort mode (1=effort, 2=velocity, 0=position)
ros2 topic pub --once /arm_left_motion_controller_interface/set_mode std_msgs/msg/Int32 "{data: 1}"
```

### Testing

There is currently no automated test suite (`colcon test`) in this repo. Validate changes manually:

1. Build (`colcon build`) and confirm there are no compiler/linker errors.
2. Launch the Gazebo Classic scenario closest to your change and confirm with `ros2 control list_controllers` that the expected controller(s) report `active`.
3. Publish a `JointCommand` (see above) and confirm the arm moves as expected / stays within joint limits.
4. If touching `motion_controller_interface`, exercise the mode-switch and command-timeout paths: switch to effort/velocity, then stop publishing commands and confirm it auto-reverts to position after `command_timeout` seconds.
