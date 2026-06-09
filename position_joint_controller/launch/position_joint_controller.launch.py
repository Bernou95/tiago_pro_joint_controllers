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
# Brings up the TiagoPro and spawns position controller(s) for one or both arms.
#
# Usage:
#   ros2 launch position_joint_controller position_joint_controller.launch.py \
#       robot_ip:=<IP> arm_side:=both   # (default) both arms
#   ros2 launch position_joint_controller position_joint_controller.launch.py \
#       robot_ip:=<IP> arm_side:=left
#   ros2 launch position_joint_controller position_joint_controller.launch.py \
#       robot_ip:=<IP> arm_side:=right

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_pal.include_utils import include_launch_py_description


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
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        description='IP address of the TiagoPro robot',
    )
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='tiago_pro',
        description='Robot model (tiago_pro, ...)',
    )
    arm_side_arg = DeclareLaunchArgument(
        'arm_side',
        default_value='both',
        choices=['left', 'right', 'both'],
        description='Which arm(s) to control: left, right, or both (default).',
    )
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='false',
        description='Load TiagoPro gripper',
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware instead of real robot',
    )

    bringup = include_launch_py_description(
        pkg_name='tiago_pro_bringup',
        paths=['launch', 'tiago_pro.launch.py'],
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_type': LaunchConfiguration('robot_type'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        },
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_type_arg,
        arm_side_arg,
        load_gripper_arg,
        use_fake_hardware_arg,
        bringup,
        OpaqueFunction(function=spawn_controllers),
    ])
