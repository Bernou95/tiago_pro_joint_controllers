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

# Launch file for the PositionJointController in Gazebo simulation.
#
# The position command interface is always exposed in Gazebo regardless of the
# gazebo_effort xacro flag. The spawner --param-file mechanism sets the type
# on the controller_manager dynamically before load_controller is called.
#
# Usage:
#   ros2 launch position_joint_controller position_joint_controller_gazebo.launch.py

import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_robot_description(context: LaunchContext, robot_type, load_gripper, end_effector):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    end_effector_str = context.perform_substitution(end_effector)

    tiago_pro_xacro_file = os.path.join(
        get_package_share_directory('tiago_pro_gazebo_bringup'),
        'urdf',
        'tiago_pro_arm.gazebo.xacro'
    )

    robot_description_config = xacro.process_file(
        tiago_pro_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': end_effector_str,
            # gazebo_effort is NOT set: position interface is always available in Gazebo
        },
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_config.toxml()}]
    )

    return [robot_state_publisher]


def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument('robot_type', default_value='tiago_pro',
                                           description='Robot model (tiago_pro, …)')
    load_gripper_arg = DeclareLaunchArgument('load_gripper', default_value='false',
                                             description='Load TiagoPro gripper')
    end_effector_arg = DeclareLaunchArgument('end_effector', default_value='tiago_pro_hand',
                                             description='End-effector id')

    robot_type = LaunchConfiguration('robot_type')
    load_gripper = LaunchConfiguration('load_gripper')
    end_effector = LaunchConfiguration('end_effector')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, end_effector]
    )

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('tiago_pro_description')
    )
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    rviz_file = os.path.join(
        get_package_share_directory('tiago_pro_description'), 'rviz', 'visualize_tiago_pro.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('position_joint_controller'), 'config', 'controllers_gazebo.yaml']
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    spawn_position_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_joint_controller',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['joint_states'], 'rate': 30}],
    )

    return LaunchDescription([
        robot_type_arg,
        load_gripper_arg,
        end_effector_arg,
        gazebo,
        robot_state_publisher,
        rviz,
        spawn,
        bridge,
        joint_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[spawn_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_position_joint_controller],
            )
        ),
    ])
