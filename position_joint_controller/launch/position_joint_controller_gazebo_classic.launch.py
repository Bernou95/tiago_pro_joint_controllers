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


import os
import yaml
import tempfile
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_pal.include_utils import (
    include_scoped_launch_py_description,
    include_launch_py_description,
)
from launch_pal.actions import CheckPublicSim

from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
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
    #gazebo_version: DeclareLaunchArgument = CommonArgs.gazebo_version


def private_navigation(context, *args, **kwargs):
    actions = []
    base_type = read_launch_argument('base_type', context)
    camera_model = read_launch_argument('camera_model', context)
    docking = read_launch_argument('docking', context)
    advanced_navigation = read_launch_argument('advanced_navigation', context)
    use_sim_time = read_launch_argument('use_sim_time', context)
    rviz_cfg_pkg = base_type + '_2dnav'
    if advanced_navigation == 'True':
        rviz_cfg_pkg = base_type + '_advanced_2dnav'

    robot_info = {
        "robot_info_publisher": {
            "ros__parameters": {
                "robot_type": "tiago_pro",
                "base_type": base_type,
                "laser_model": "no-laser",
                "camera_model": camera_model,
                "advanced_navigation": (advanced_navigation == 'False'),
                "has_dock": (docking == 'False'),
                "use_sim_time": (use_sim_time == 'True'),
            }
        }
    }

    temp_yaml = tempfile.mkdtemp()
    temp_robot_info = os.path.join(temp_yaml, '99_robot_info.yaml')
    with open(temp_robot_info, 'w') as temp_robot_info_file:
        yaml.safe_dump(robot_info, temp_robot_info_file)

    # Robot Info Publisher
    robot_info_env = SetEnvironmentVariable(
        name='ROBOT_INFO_PATH',
        value=temp_yaml,
    )
    actions.append(robot_info_env)

    robot_info_publisher = Node(
        package='robot_info_publisher',
        executable='robot_info_publisher',
        name='robot_info_publisher',
        output='screen',
    )
    actions.append(robot_info_publisher)

    return actions


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    # Set use_sim_time to True
    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")
    launch_description.add_action(set_sim_time)

    # Shows error if is_public_sim is not set to True when using public simulation
    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    robot_name = "tiago_pro"
    packages = ["tiago_pro_description", "pal_sea_arm_description",
                "omni_base_description", "pal_pro_gripper_description",
<<<<<<< HEAD
                "tiago_pro_head_description", "allegro_hand_description",
                "pal_urdf_utils"]

=======
                "tiago_pro_head_description"]
>>>>>>> origin/main

    model_path = get_model_paths(packages)

    gazebo_model_path_env_var = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", model_path)

    gazebo = include_scoped_launch_py_description(
        pkg_name="position_joint_controller",
        paths=["launch", "pal_gazebo.launch.py"],
        env_vars=[gazebo_model_path_env_var],
        launch_arguments={
            "world_name":  "empty",
            "model_paths": packages,
            "resource_paths": packages,
            #"debug":"True",
        })

    launch_description.add_action(gazebo)
<<<<<<< HEAD
    
    move_group = include_scoped_launch_py_description(
        pkg_name="tiago_pro_moveit_config",
        paths=["launch", "move_group.launch.py"],
        launch_arguments={
            "robot_name": robot_name,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "base_type": launch_args.base_type,
            "arm_type_right": launch_args.arm_type_right,
            "arm_type_left": launch_args.arm_type_left,
            "end_effector_right": launch_args.end_effector_right,
            "end_effector_left": launch_args.end_effector_left,
            "ft_sensor_right": launch_args.ft_sensor_right,
            "ft_sensor_left": launch_args.ft_sensor_left
        },
        condition=IfCondition(LaunchConfiguration("moveit")))

    launch_description.add_action(move_group)
=======
>>>>>>> origin/main


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
            "gazebo_version": "classic"
        }
    )
    launch_description.add_action(robot_state_publisher)

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('position_joint_controller'), 'config', 'controllers_gazebo.yaml']
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
        output='screen',
    )

    spawn_position_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_joint_controller', '--param-file', controllers_yaml],
        output='screen',
    )

    launch_description.add_action(spawn_joint_state_broadcaster)
    launch_description.add_action(spawn_position_joint_controller)

    return


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    if "GAZEBO_MODEL_PATH" in environ:
        model_paths += pathsep + environ["GAZEBO_MODEL_PATH"]

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ""
    for package_name in packages_names:
        if resource_paths != "":
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    if "GAZEBO_RESOURCE_PATH" in environ:
        resource_paths += pathsep + environ["GAZEBO_RESOURCE_PATH"]

    return resource_paths


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

<<<<<<< HEAD
    return ld
=======
    return ld
>>>>>>> origin/main
