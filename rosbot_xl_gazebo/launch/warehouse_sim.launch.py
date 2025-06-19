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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose

from ament_index_python.packages import get_package_share_directory
import os 

def launch_setup(context, *args, **kwargs):
    camera_model = context.perform_substitution(LaunchConfiguration("camera_model"))
    lidar_model = context.perform_substitution(LaunchConfiguration("lidar_model"))
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    x = LaunchConfiguration("x", default="1.88").perform(context)
    y = LaunchConfiguration("y", default="-9.8").perform(context)
    z = LaunchConfiguration("z", default="0.0").perform(context)
    roll = LaunchConfiguration("roll", default="0.0").perform(context)
    pitch = LaunchConfiguration("pitch", default="0.0").perform(context)
    yaw = LaunchConfiguration("yaw", default="1.57").perform(context)

    gz_args = f"--headless-rendering -s -v 4 -r {world}" if eval(headless) else f"-r {world}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "True",
        }.items(),
    )

    logger = LogInfo(msg=["Launching robot simulation"])
    robots_list = ParseMultiRobotPose("robots").value()
    init_pose = {
        "x": x,
        "y": y,
        "z": z,
        "roll": roll, 
        "pitch": pitch,
        "yaw": yaw,
    }

    spawn_log = LogInfo(
        msg=[f"Launching robot with init_pose = {str(init_pose)}"]
    )
    

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_xl_gazebo"),
                    "launch",
                    "spawn.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": "False",
            "use_sim": "True",
            "lidar_model": lidar_model,
            "camera_model": camera_model,
            "simulation_engine": "ignition-gazebo",
            "x": TextSubstitution(text=str(init_pose["x"])),
            "y": TextSubstitution(text=str(init_pose["y"])),
            "z": TextSubstitution(text=str(init_pose["z"])),
            "roll": TextSubstitution(text=str(init_pose["roll"])),
            "pitch": TextSubstitution(text=str(init_pose["pitch"])),
            "yaw": TextSubstitution(text=str(init_pose["yaw"])),
        }.items(),
    )

    spawn_group = GroupAction(
        actions=[
            spawn_log,
            spawn_robot
        ]
    )

    return [logger, gz_sim, spawn_group]

def generate_launch_description():

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
    )

    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="orbbec_astra",
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

    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="slamtec_rplidar_s3",
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

    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
    )


    warehouse_world_dir = get_package_share_directory('warehouse_world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(warehouse_world_dir, 'worlds', 'zm_robot_warehouse.sdf'),
        description='Full path to world model file to load'
    )

    model_path = os.path.join(warehouse_world_dir, 'models')

    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo Ignition in the headless mode",
    )
    
    

    return LaunchDescription(
        [
            declare_mecanum_arg,
            declare_lidar_model_arg,
            declare_camera_model_arg,
            declare_include_camera_mount_arg,
            declare_world_cmd,
            declare_headless_arg,
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=launch_setup),
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path)
        ]
    )
