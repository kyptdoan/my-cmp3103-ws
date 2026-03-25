Level 3 commands

ros2 launch uol_tidybot tidybot.launch.py world:=level_2_1.world

ros2 launch nav2_bringup localization_launch.py use_sim_time:=true map:=/workspaces/my-cmp3103-ws/cube_pushing/map/room_map.yaml params_file:=/workspaces/my-cmp3103-ws/cube_pushing/config/nav2_params.yaml

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/workspaces/my-cmp3103-ws/cube_pushing/config/nav2_params.yaml

ros2 run rviz2 rviz2 -d /workspaces/my-cmp3103-ws/cube_pushing/rviz/complexity_3.rviz


