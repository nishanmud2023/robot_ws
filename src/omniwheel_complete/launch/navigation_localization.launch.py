#!/usr/bin/env python3
"""
Navigation + AMCL localization on a previously saved map.

This is the "use the map" counterpart to navigation_with_slam.launch.py:

    navigation_with_slam.launch.py   ->  slam_toolbox builds a NEW map
    navigation_localization.launch.py ->  map_server serves a SAVED map,
                                          amcl localises the robot in it

Both files launch the same nav2 navigation stack (planner, controller,
bt_navigator, costmaps) and the same RViz config. The only difference is
what supplies the /map topic and the map -> odom transform.

Usage (sim, after Gazebo is already running):

    ros2 launch omniwheel_complete navigation_localization.launch.py

Usage on real hardware (bringupomni_real.launch.py running on the Pi):

    ros2 launch omniwheel_complete navigation_localization.launch.py \
        use_sim_time:=False

Optional args:
    map:=/path/to/other_map.yaml   use a different saved map
    rviz:=false                    don't open RViz
    use_sim_time:=False            real robot / real clock

The map defaults to the one installed with this package
(share/omniwheel_complete/maps/my_map.yaml), so the command is identical
in sim and on hardware — only the map file's contents differ. The sim map
of the hospital world is useless on the real robot: remap the real room
with navigation_with_slam.launch.py first.

After launching you MUST click "2D Pose Estimate" in RViz and drag on the
map at the robot's real position — amcl starts with no idea where it is and
will keep printing "AMCL cannot publish a pose..." until you do. Then use
"2D Nav Goal" to send it somewhere.

Never run this at the same time as navigation_with_slam.launch.py: both
slam_toolbox and amcl publish the map -> odom transform, and having both
alive corrupts the TF tree.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_omniwheel_complete = get_package_share_directory('omniwheel_complete')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_omniwheel_complete)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # The saved map, installed with the package under share/.../maps/.
    # Produced by:
    #   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    #       "{name: {data: 'my_map'}}"
    # then moved into src/omniwheel_complete/maps/. Keep the .yaml and .pgm
    # together — the yaml references the pgm by a relative filename.
    #
    # Living inside the package (rather than in the home directory) means
    # the same command works on the laptop and on the Pi, with no absolute
    # paths to fix up.
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_omniwheel_complete, 'maps', 'my_map.yaml'),
        description='Full path to the saved map .yaml file'
    )

    # ------------------------------------------------------------------
    # Config / include paths
    # ------------------------------------------------------------------
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    # Same params file the slam version feeds to nav2, so the planner and
    # controller behave identically in both modes.
    navigation_params_path = os.path.join(
        pkg_omniwheel_complete,
        'config',
        'navigation.yaml'
    )

    # amcl reads its parameters from here. If config/navigation.yaml has no
    # "amcl:" section, amcl silently falls back to its built-in defaults
    # (base_frame_id: base_footprint, scan_topic: scan) — which is what was
    # running when this setup was first tested, so it works either way.
    # Override with localization_params_file:=... to point somewhere else,
    # e.g. /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml
    localization_params_arg = DeclareLaunchArgument(
        'localization_params_file',
        default_value=navigation_params_path,
        description='Parameters file for map_server and amcl'
    )

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_omniwheel_complete, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    # ------------------------------------------------------------------
    # map_server + amcl  (replaces slam_toolbox)
    #
    # map_server: loads the .pgm/.yaml and republishes it, unchanging, on
    #             /map. It does not know where the robot is.
    # amcl:       particle filter that matches the live /scan against that
    #             static map to produce the map -> odom transform.
    # ------------------------------------------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('localization_params_file'),
        }.items()
    )

    # ------------------------------------------------------------------
    # nav2 navigation stack — planner, controller, bt_navigator,
    # behaviour server, both costmaps. Identical to the slam version.
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(map_arg)
    launchDescriptionObject.add_action(localization_params_arg)

    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
