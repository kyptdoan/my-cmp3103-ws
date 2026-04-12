#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ------- Simulation --------
    tidybot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('uol_tidybot'), 'launch', 'tidybot.launch.py')
        )
    )

    # ------- SLAM Toolbox -----------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(get_package_share_directory('cube_pushing'), 'config', 'slam_params.yaml')
        }.items()
    )

    # ------- Nav2 ----------
    # use default nav2 params as the objective of task 1 and 2 does not require precise navigation
    nav2_params = os.path.join(
        get_package_share_directory('cube_pushing'),
        'config',
        'nav2_params_real_robot.yaml'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )

    # -------- RViz -----------
    rviz = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[cube_pushing] Launching RViz...'),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory('cube_pushing'), 'rviz', 'real_robot.rviz')],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[Task_1] Launching simulation...'),
        tidybot,
        LogInfo(msg='[Real_Robot] Launching SLAM...'),
        slam,
        LogInfo(msg='[Real_Robot] Launching Nav2...'),
        nav2,
        LogInfo(msg='[Real_Robot] Launching Rviz...'),
        rviz,
    ])