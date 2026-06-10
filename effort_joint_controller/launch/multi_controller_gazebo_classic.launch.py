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

# Multi-controller Gazebo Classic launch — single or dual arm.
#
# For each controlled arm spawns:
#   arm_{side}_position_joint_controller          (active — default safe mode)
#   arm_{side}_velocity_joint_controller          (loaded inactive)
#   arm_{side}_effort_joint_controller            (loaded inactive)
#   arm_{side}_gravity_compensation_controller    (loaded inactive — PAL's launcher)
#   arm_{side}_motion_controller_interface        (always active — coordinator)
# Plus one shared joint_state_broadcaster.
#
# Switch arm mode at runtime
# (std_msgs/Int32: 0=position  1=effort  2=velocity  3=gravity_compensation):
#   ros2 topic pub --once /arm_left_motion_controller_interface/set_mode  \
#       std_msgs/msg/Int32 "{data: 1}"
#   ros2 topic pub --once /arm_right_motion_controller_interface/set_mode \
#       std_msgs/msg/Int32 "{data: 1}"
#
# Usage:
#   ros2 launch effort_joint_controller multi_controller_gazebo_classic.launch.py \
#       arm_side:=both    # (default)
#   ros2 launch effort_joint_controller multi_controller_gazebo_classic.launch.py \
#       arm_side:=left
#   ros2 launch effort_joint_controller multi_controller_gazebo_classic.launch.py \
#       arm_side:=right

import os
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


def spawn_controllers(context):
    arm_side = context.perform_substitution(LaunchConfiguration('arm_side'))
    pkg = FindPackageShare('effort_joint_controller')

    def _spawner(name, yaml_path, inactive=False):
        args = [name, '--param-file', yaml_path]
        if inactive:
            args.insert(1, '--inactive')
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=args,
            output='screen',
        )

    def _grav_comp(side):
        # Re-uses PAL's gravity_compensation_controller.launch.py from the docker image.
        # That launcher loads the controller as a ros2_control plugin; it is expected
        # to be inactive after loading (matches PAL's joy_teleop change_controllers
        # pattern) so the MotionControllerInterface can activate it via set_mode=3.
        wrist_arg = context.perform_substitution(
            LaunchConfiguration(f'wrist_model_{side}'))
        return include_scoped_launch_py_description(
            pkg_name='pal_sea_arm_controller_configuration',
            paths=['launch', 'gravity_compensation_controller.launch.py'],
            launch_arguments={
                'side': side,
                'root_link': 'torso_lift_link',
                'wrist_model': wrist_arg,
            })

    def _arm_set(side, yaml_path):
        return [
            _spawner(f'arm_{side}_position_joint_controller', yaml_path),
            _spawner(f'arm_{side}_velocity_joint_controller', yaml_path, inactive=True),
            _spawner(f'arm_{side}_effort_joint_controller', yaml_path, inactive=True),
            _grav_comp(side),
            _spawner(f'arm_{side}_motion_controller_interface', yaml_path),
        ]

    if arm_side == 'left':
        yaml = PathJoinSubstitution([pkg, 'config', 'left_arm_multi_controllers_gazebo.yaml'])
        return [_spawner('joint_state_broadcaster', yaml)] + _arm_set('left', yaml)
    elif arm_side == 'right':
        yaml = PathJoinSubstitution([pkg, 'config', 'right_arm_multi_controllers_gazebo.yaml'])
        return [_spawner('joint_state_broadcaster', yaml)] + _arm_set('right', yaml)
    else:  # both
        yaml = PathJoinSubstitution([pkg, 'config', 'dual_arm_multi_controllers_gazebo.yaml'])
        return (
            [_spawner('joint_state_broadcaster', yaml)]
            + _arm_set('left', yaml)
            + _arm_set('right', yaml)
        )


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

    launch_description.add_action(OpaqueFunction(function=spawn_controllers))


def generate_launch_description():
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    ld.add_action(DeclareLaunchArgument(
        'arm_side',
        default_value='both',
        choices=['left', 'right', 'both'],
        description='Which arm(s) to control: left, right, or both (default).',
    ))
    declare_actions(ld, launch_arguments)
    return ld
