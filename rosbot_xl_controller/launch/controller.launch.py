#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
# Copyright 2024 Husarion sp. z o.o.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    controller_manager_name = "controller_manager"

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        name="robot_controller_spawner",
        executable="spawner",
        arguments=[
            "rosbot_xl_base_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    extendable_contoller_spawner = Node(
        package='controller_manager',
        name="extendable_mount_controller_spawner",
        executable='spawner',
        arguments=[
            'extendable_mount_controller',
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ]
    )
    delay_extendable_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[extendable_contoller_spawner],
        )
    )



    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        name="imu_broadcaster_spawner",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )
    return [
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
        delay_extendable_controller_spawner_after_joint_state_broadcaster_spawner,
    ]


def generate_launch_description():

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="None",
        description="Add camera model to the robot URDF",
        choices=[
            "None",
            "intel_realsense_d435",
            "orbbec_astra",
            "stereolabs_zed",
            "stereolabs_zedm",
            "stereolabs_zed2",
            "stereolabs_zed2i",
            "stereolabs_zedx",
            "stereolabs_zedxm",
        ],
    )

    lidar_model = LaunchConfiguration("lidar_model")
    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="None",
        description="Add LiDAR model to the robot URDF",
        choices=[
            "None",
            "slamtec_rplidar_a2",
            "slamtec_rplidar_a3",
            "slamtec_rplidar_s1",
            "slamtec_rplidar_s2",
            "slamtec_rplidar_s3",
            "velodyne_puck",
        ],
    )

    include_camera_mount = LaunchConfiguration("include_camera_mount")
    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="ignition-gazebo",
        description="Which simulation engine will be used",
    )

    controller_config_name = 'diff_drive_controller.yaml'

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_xl_controller"),
            "config",
            controller_config_name,
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot_xl.urdf.xacro",
                ]
            ),
            " controller_config_file:=",
            controller_config_path,
            " lidar_model:=",
            lidar_model,
            " camera_model:=",
            camera_model,
            " include_camera_mount:=",
            include_camera_mount,
            " use_sim:=",
            use_sim,
            " simulation_engine:=",
            simulation_engine,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription(
        [
            declare_lidar_model_arg,
            declare_camera_model_arg,
            declare_include_camera_mount_arg,
            declare_use_sim_arg,
            declare_simulation_engine_arg,
            # SetParameter(name="use_sim_time", value=use_sim),
            robot_state_pub_node,
            OpaqueFunction(function=launch_setup),

        
        ]
    )
