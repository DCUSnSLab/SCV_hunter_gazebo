#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the use_sim_time argument
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Launch configuration for use_sim_time
    use_sim_time_config = LaunchConfiguration('use_sim_time')

    # Set Gazebo plugin path for velodyne plugin
    scv_pkg_path = get_package_share_directory('scv_robot_gazebo')
    plugin_lib_path = os.path.join(os.path.dirname(scv_pkg_path), 'lib')
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        [plugin_lib_path, ':', os.environ.get('GAZEBO_PLUGIN_PATH', '')]
    )
    
    # K-City world file path
    k_city_pkg_path = get_package_share_directory('k_city_gazebo')
    world_file = os.path.join(k_city_pkg_path, 'worlds', 'k_city.world')
    
    # Gazebo parameters file
    gazebo_params_file = os.path.join(get_package_share_directory("scv_robot_gazebo"), 'config', 'gazebo_params.yaml')
    
    # Launch Gazebo with K-City world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("scv_robot_gazebo"), "urdf", 'hunter_with_one_box.urdf.xacro']
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time_config}]
    )

    # Spawn entity in K-City with longer timeout
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'scv_robot', '-x', '0', '-y', '0', '-z', '0.5', '-timeout', '60'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_config}]
    )

    # Add RViz with hunter description config
    hunter_description_path = os.path.join(
        get_package_share_directory('hunter_description'))
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(hunter_description_path, 'rviz/robot_view.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time_config}]
    )

    # Controller loading (Hunter style)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_like_controller'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        gazebo_plugin_path,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_controller],
            )
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz,
    ])