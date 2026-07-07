# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

# Real-robot launcher for PositionJointController.
#
# Spawns position controller(s) for one or both arms onto the controller_manager
# that is already running on the robot (brought up by PAL's own system modules
# at boot). This package is deployed and run locally on the robot, so no robot
# IP or remote bringup is needed here.
#
# Usage:
#   ros2 launch position_joint_controller position_joint_controller.launch.py \
#       arm_side:=both   # (default) both arms
#   ros2 launch position_joint_controller position_joint_controller.launch.py \
#       arm_side:=left
#   ros2 launch position_joint_controller position_joint_controller.launch.py \
#       arm_side:=right

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_controllers(context):
    arm_side = context.perform_substitution(LaunchConfiguration('arm_side'))
    pkg = FindPackageShare('position_joint_controller')
    actions = []

    def _spawner(controller_name, yaml_path):
        return Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{controller_name}',
            arguments=[
                controller_name,
                '--controller-manager-timeout', '30',
                '--param-file', yaml_path,
            ],
            output='screen',
        )

    if arm_side == 'left':
        yaml = PathJoinSubstitution([pkg, 'config', 'left_arm_controllers.yaml'])
        actions.append(_spawner('arm_left_position_joint_controller', yaml))
    elif arm_side == 'right':
        yaml = PathJoinSubstitution([pkg, 'config', 'right_arm_controllers.yaml'])
        actions.append(_spawner('arm_right_position_joint_controller', yaml))
    else:  # both
        yaml_left = PathJoinSubstitution([pkg, 'config', 'left_arm_controllers.yaml'])
        yaml_right = PathJoinSubstitution([pkg, 'config', 'right_arm_controllers.yaml'])
        actions.append(_spawner('arm_left_position_joint_controller', yaml_left))
        actions.append(_spawner('arm_right_position_joint_controller', yaml_right))

    return actions


def generate_launch_description():
    arm_side_arg = DeclareLaunchArgument(
        'arm_side',
        default_value='both',
        choices=['left', 'right', 'both'],
        description='Which arm(s) to control: left, right, or both (default).',
    )

    return LaunchDescription([
        arm_side_arg,
        OpaqueFunction(function=spawn_controllers),
    ])
