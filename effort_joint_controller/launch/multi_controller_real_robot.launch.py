# Copyright (c) 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Multi-controller real-robot launch — always both arms.
#
# Real-robot analog of multi_controller_gazebo_classic.launch.py. Assumes the
# robot's own controller_manager, hardware interface, and robot_state_publisher
# are already running (brought up by PAL's own system modules at boot — see
# module/arm_controllers.yaml), and only spawns this package's controllers
# onto it. Never starts Gazebo, a robot description, or a controller_manager.
#
# For each arm spawns:
#   arm_{side}_gravity_compensation_controller    (active — default mode)
#   arm_{side}_position_joint_controller          (loaded inactive)
#   arm_{side}_velocity_joint_controller          (loaded inactive)
#   arm_{side}_effort_joint_controller            (loaded inactive)
#   arm_{side}_motion_controller_interface        (always active — coordinator)
#
# Both arms are always controlled — there is no arm_side argument, since a
# robot never runs one arm on this stack and the other on PAL's stock stack.
#
# PAL's `default_controllers` module (base/torso/head/arms) does NOT need to
# be disabled before running this — it stays running. Its arm_{side}_controller
# (a JointTrajectoryController) claims only the `position` command interface,
# so it only conflicts with our position mode; effort, velocity, and
# gravity_compensation modes are conflict-free. motion_controller_interface's
# pal_arm_controller_name parameter (set in dual_arm_multi_controllers.yaml)
# releases that one claim atomically whenever position mode is requested.
# Just `pal module activate arm_controllers` (or run this file directly).
#
# Switch mode at runtime (std_msgs/Int32: 0=position 1=effort 2=velocity
# 3=gravity_compensation):
#   ros2 topic pub --once /arm_left_motion_controller_interface/set_mode  \
#       std_msgs/msg/Int32 "{data: 1}"
#   ros2 topic pub --once /arm_right_motion_controller_interface/set_mode \
#       std_msgs/msg/Int32 "{data: 1}"
#
# Usage:
#   ros2 launch effort_joint_controller multi_controller_real_robot.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_pal.param_utils import parse_parametric_yaml


def _spawner(controller_name, param_file, inactive):
    arguments = [
        controller_name,
        '--controller-manager-timeout', '30',
        '--param-file', param_file,
    ]
    if inactive:
        arguments.append('--inactive')
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{controller_name}',
        arguments=arguments,
        output='screen',
    )


def spawn_controllers(context, *args, **kwargs):
    yaml_path = os.path.join(
        get_package_share_directory('effort_joint_controller'),
        'config', 'dual_arm_multi_controllers.yaml')

    actions = []
    for side in ('left', 'right'):
        actions.append(_spawner(
            f'arm_{side}_position_joint_controller', yaml_path, inactive=True))
        actions.append(_spawner(
            f'arm_{side}_velocity_joint_controller', yaml_path, inactive=True))
        actions.append(_spawner(
            f'arm_{side}_effort_joint_controller', yaml_path, inactive=True))

        # PAL's own gravity_compensation_controller.launch.py always spawns
        # --inactive with no option to start active, so its parameter
        # resolution step is replicated here directly (same source yaml,
        # same ${ARM_SIDE_PREFIX} substitution) rather than reused wholesale.
        grav_param_file = parse_parametric_yaml(
            source_files=[os.path.join(
                get_package_share_directory('pal_sea_arm_controller_configuration'),
                'config', 'arm_gravity_compensation_controller_effort.yaml')],
            param_rewrites={"ARM_SIDE_PREFIX": f"arm_{side}"})
        actions.append(_spawner(
            f'arm_{side}_gravity_compensation_controller', grav_param_file, inactive=False))

        actions.append(_spawner(
            f'arm_{side}_motion_controller_interface', yaml_path, inactive=False))

    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=spawn_controllers))
    return ld
