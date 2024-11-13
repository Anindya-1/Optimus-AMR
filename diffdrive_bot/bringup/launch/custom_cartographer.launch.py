import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess,\
                           IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro


def generate_launch_description():
    
    pkg_name = 'diffdrive_bot'

    rviz_config = os.path.join(
      get_package_share_directory(pkg_name),
      'rviz',
      'cartographer.rviz'
    )

    slam_config = os.path.join(
      get_package_share_directory(pkg_name),
      'config',
      'mapper_params_online_async.yaml'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],      
    )
    
    scan_update = Node(
            package="sensor_manip",
            executable="scan_update"
        )
  
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
                                       '/online_async_launch.py']),
        launch_arguments={'slam_params_file': slam_config,
                          'use_sim_time': 'true'
                          }.items()
    )

    return LaunchDescription([
        scan_update,
        slam,
        # rviz_node        
    ])