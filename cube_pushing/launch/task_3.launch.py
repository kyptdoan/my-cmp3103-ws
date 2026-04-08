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
        ),
        launch_arguments={
            'world': 'level_2_1.world',
        }.items()
    )

    # ------- Nav2 using pre-built map (delayed to allow simulation to start) ----------
    nav2_params = os.path.join(
        get_package_share_directory('cube_pushing'),
        'config',
        'nav2_params_task_3.yaml'
    )

    nav2 = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[Task_3] Launching Nav2...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': '/workspaces/my-cmp3103-ws/cube_pushing/map/room_map.yaml',
                    'params_file': nav2_params,
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # -------- RViz (delayed to allow Nav2 and AMCL to publish TF) -----------
    rviz = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[Task_3] Launching RViz...'),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory('cube_pushing'), 'rviz', 'level_3.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # -------- Generate objects (delayed to allow simulation to settle) ----------
    generate_objects = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[Task_3] Generating objects...'),
            ExecuteProcess(
                cmd=['ros2', 'run', 'uol_tidybot', 'generate_objects',
                     '--ros-args',
                     '-p', 'n_objects:=1',
                     '-p', 'cx:=0.7',
                     '-p', 'cy:=1.15',
                     '-p', 'spread:=0.1'],
                output='screen'
            )
        ]
    )

    # --------- Level 3 cube pushing nodes ---------
    estimate_position_node = Node(package='cube_pushing', executable='estimate_position', name='estimate_position_node', output='screen')
    go_to_pre_pushing_pose_node = Node(package='cube_pushing', executable='go_to_pre_pushing_pose', name='go_to_pre_pushing_pose_node', output='screen')

    # --------- State machine -----------
    state_machine = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[Task_3] Starting state machine...'),
            Node(package='cube_pushing', executable='state_machine_task_3', name='state_machine_node', output='screen'),
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[Task_3] Launching simulation...'),
        tidybot,
        LogInfo(msg='[Task_3] Launching Nav2...'),
        nav2,
        LogInfo(msg='[Task_3] Launching RViz...'),
        rviz,
        LogInfo(msg='[Task_3] Generating objects...'),
        generate_objects,
        LogInfo(msg='[Task_3] Launching cube pushing nodes...'),
        estimate_position_node,
        go_to_pre_pushing_pose_node,
        state_machine,
    ])