# Author: Enrico Mendez
# Date: 04 Marzo 2024
# Description: Realsense camera launch with align depth propoerties
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import PushRosNamespace



def generate_launch_description():


    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py')),
                launch_arguments = {'align_depth.enable': 'true'}.items()
    )

    l_d = LaunchDescription([launch_include])

    return l_d


