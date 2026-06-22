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

# Gazebo Classic launcher for PositionJointController.
#
# Controls one or both TiagoPro arms by spawning arm_left_ / arm_right_ prefixed
# controller instances.  The arm_side argument selects which arm(s) to spawn:
#
#   ros2 launch position_joint_controller position_joint_controller_gazebo_classic.launch.py \
#       arm_side:=both     # (default) spawn both arms
#   ros2 launch position_joint_controller position_joint_controller_gazebo_classic.launch.py \
#       arm_side:=left
#   ros2 launch position_joint_controller position_joint_controller_gazebo_classic.launch.py \
#       arm_side:=right

import os
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.actions import CheckPublicSim
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs
from dataclasses import dataclass
from launch_ros.actions import Node


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

    navigation: DeclareLaunchArgument = CommonArgs.navigation
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    slam: DeclareLaunchArgument = CommonArgs.slam
    docking: DeclareLaunchArgument = CommonArgs.docking
    moveit: DeclareLaunchArgument = CommonArgs.moveit
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    tuck_arm: DeclareLaunchArgument = CommonArgs.tuck_arm
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim


def spawn_controllers(context):
    """Spawn the correct controller set based on arm_side."""
    arm_side = context.perform_substitution(LaunchConfiguration('arm_side'))
    pkg = FindPackageShare('position_joint_controller')
    actions = []

    # Always spawn the broadcaster first (config file provides the joint list).
    def _spawner(controller_name, yaml_path):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name, '--param-file', yaml_path],
            output='screen',
        )

    if arm_side in ('left', 'both'):
        yaml_left = PathJoinSubstitution([pkg, 'config', 'left_arm_gazebo.yaml'])
        actions.append(_spawner('joint_state_broadcaster', yaml_left))
        actions.append(_spawner('arm_left_position_joint_controller', yaml_left))

    if arm_side in ('right', 'both'):
        yaml_right = PathJoinSubstitution([pkg, 'config', 'right_arm_gazebo.yaml'])
        if arm_side == 'right':
            # broadcaster not yet spawned
            actions.append(_spawner('joint_state_broadcaster', yaml_right))
        actions.append(_spawner('arm_right_position_joint_controller', yaml_right))

    if arm_side == 'both':
        # For dual arm, re-use the dual_arm config for the broadcaster so it
        # covers all 14 joints, then override the two controller spawns.
        yaml_dual = PathJoinSubstitution([pkg, 'config', 'dual_arm_gazebo.yaml'])
        actions = [
            _spawner('joint_state_broadcaster', yaml_dual),
            _spawner('arm_left_position_joint_controller', yaml_dual),
            _spawner('arm_right_position_joint_controller', yaml_dual),
        ]

    return actions


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")
    launch_description.add_action(set_sim_time)

    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    packages = ["tiago_pro_description", "pal_sea_arm_description",
                "omni_base_description", "pal_pro_gripper_description",
                "tiago_pro_head_description", "allegro_hand_description",
                "pal_urdf_utils"]

    model_path = get_model_paths(packages)
    gazebo_model_path_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

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
            "gazebo_version": "classic",
        }
    )
    launch_description.add_action(robot_state_publisher)

    # Mobile base controller
    launch_description.add_action(OpaqueFunction(function=launch_mobile_base_controller))

    launch_description.add_action(OpaqueFunction(function=spawn_controllers))

def launch_mobile_base_controller(context, *args, **kwargs):
    # PAL's mobile_base_controller.launch.py gates the OmniDriveController behind
    # UnlessCondition(use_sim_time), so it never spawns in simulation. We bypass
    # that gate and spawn the controller directly so the wheel joints are commanded
    # to zero velocity, preventing arm reaction forces from rotating the base.
    base_type = context.perform_substitution(LaunchConfiguration('base_type'))
    pkg = get_package_share_directory(base_type + '_controller_configuration')
    yaml_path = os.path.join(pkg, 'config', 'mobile_base_controller.yaml')

    return [Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_base_controller', '--param-file', yaml_path],
        output='screen',
    )]

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
