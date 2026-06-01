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

# Multi-controller Gazebo Classic launch.
#
# Spawns:
#   - joint_state_broadcaster         (always active)
#   - position_joint_controller        (active — default safe mode)
#   - effort_joint_controller          (loaded inactive — activated via MotionControllerInterface)
#   - motion_controller_interface      (always active — coordinator/switcher)
#
# Switch to effort mode at runtime:
#   ros2 topic pub --once /motion_controller_interface/set_mode std_msgs/msg/Int32 "{data: 1}"
# Switch back to position mode:
#   ros2 topic pub --once /motion_controller_interface/set_mode std_msgs/msg/Int32 "{data: 0}"

import os
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs
from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    tool_changer_right: DeclareLaunchArgument = TiagoProArgs.tool_changer_right
    tool_changer_left: DeclareLaunchArgument = TiagoProArgs.tool_changer_left
    wrist_model_right: DeclareLaunchArgument = TiagoProArgs.wrist_model_right
    wrist_model_left: DeclareLaunchArgument = TiagoProArgs.wrist_model_left
    camera_model: DeclareLaunchArgument = TiagoProArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoProArgs.laser_model

    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep
        package_path = get_package_prefix(package_name)
        model_paths += os.path.join(package_path, "share")
    if "GAZEBO_MODEL_PATH" in environ:
        model_paths += pathsep + environ["GAZEBO_MODEL_PATH"]
    return model_paths


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    packages = [
        "tiago_pro_description", "pal_sea_arm_description",
        "omni_base_description", "pal_pro_gripper_description",
        "tiago_pro_head_description", "allegro_hand_description",
        "pal_urdf_utils",
    ]

    gazebo_model_path_env_var = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", get_model_paths(packages))

    gazebo = include_scoped_launch_py_description(
        pkg_name="position_joint_controller",
        paths=["launch", "pal_gazebo.launch.py"],
        env_vars=[gazebo_model_path_env_var],
        launch_arguments={
            "world_name": "empty",
            "model_paths": packages,
            "resource_paths": packages,
        })
    launch_description.add_action(gazebo)

    robot_spawn = include_scoped_launch_py_description(
        pkg_name="tiago_pro_gazebo",
        paths=["launch", "robot_spawn.launch.py"])
    launch_description.add_action(robot_spawn)

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='tiago_pro_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={
            "arm_type_right": launch_args.arm_type_right,
            "arm_type_left": launch_args.arm_type_left,
            "base_type": launch_args.base_type,
            "use_sim_time": "True",
            "is_public_sim": "True",
        }
    )
    launch_description.add_action(robot_state_publisher)

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('effort_joint_controller'), 'config', 'multi_controllers_gazebo.yaml']
    )

    # Always-active: publishes joint states.
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
        output='screen',
    )

    # Default active controller: holds the arm safely in position.
    spawn_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_joint_controller', '--param-file', controllers_yaml],
        output='screen',
    )

    # Loaded but inactive: activated by MotionControllerInterface on demand.
    spawn_velocity_controller_inactive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'velocity_joint_controller',
            '--inactive',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    spawn_effort_controller_inactive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'effort_joint_controller',
            '--inactive',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    # Coordinator: handles mode switching and command-timeout watchdog.
    spawn_motion_controller_interface = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motion_controller_interface', '--param-file', controllers_yaml],
        output='screen',
    )

    launch_description.add_action(spawn_joint_state_broadcaster)
    launch_description.add_action(spawn_position_controller)
    launch_description.add_action(spawn_velocity_controller_inactive)
    launch_description.add_action(spawn_effort_controller_inactive)
    launch_description.add_action(spawn_motion_controller_interface)


def generate_launch_description():
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    declare_actions(ld, launch_arguments)
    return ld
