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
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(get_package_share_directory('cube_pushing'), 'config', 'slam_params.yaml')
        }.items()
    )

    # ------- Nav2 ----------
    # use default nav2 params as the objective of task 1 and 2 does not require precise navigation
    nav2_params = os.path.join(
        get_package_share_directory('cube_pushing'),
        'config',
        'nav2_params_task_1_2.yaml'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
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
                arguments=['-d', os.path.join(get_package_share_directory('cube_pushing'), 'rviz', 'level_1_2.rviz')],
                output='screen'
            )
        ]
    )

    # -------- Generate objects ----------
    generate_objects = ExecuteProcess(
        cmd=['ros2', 'run', 'uol_tidybot', 'generate_objects',
             '--ros-args',
             '-p', 'n_objects:=10',
             '-p', 'cx:=-0.7',
             '-p', 'cy:=0.55',
             '-p', 'spread:=0.5'],
        output='screen'
    )

    # --------- Cube pushing nodes ---------
    estimate_pose_node = Node(package='cube_pushing', executable='estimate_pose', name='estimate_pose_node', output='screen')
    go_to_pose_node    = Node(package='cube_pushing', executable='go_to_pose',    name='go_to_pose_node',    output='screen')
    push_a_cube_node   = Node(package='cube_pushing', executable='push_a_cube',   name='push_a_cube_node',   output='screen', parameters=[{'brown': True}])

    # --------- State machine — delay to wait for Nav2 + SLAM to be ready ------------
    state_machine = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[Task_1] Starting state machine...'),
            Node(package='cube_pushing', executable='state_machine_task_1_2', name='state_machine_node', output='screen'),
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[Task_1] Launching simulation...'),
        tidybot,
        LogInfo(msg='[Task_1] Launching SLAM...'),
        slam,
        LogInfo(msg='[Task_1] Launching Nav2...'),
        nav2,
        rviz,
        LogInfo(msg='[Task_1] Generating objects...'),
        generate_objects,
        LogInfo(msg='[Task_1] Launching cube pushing nodes...'),
        estimate_pose_node,
        go_to_pose_node,
        push_a_cube_node,
        state_machine,
    ])