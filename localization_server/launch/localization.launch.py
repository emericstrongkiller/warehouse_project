import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')

    # Declare the launch argument for the map file name
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        description='Name of the map file (without the path) to be launched')

    # Get the value of the launch argument as a LaunchConfiguration
    map_file_name = LaunchConfiguration('map_file')

    # Construct the full path to the map file using PathJoinSubstitution
    # This substitution will resolve at launch time
    map_file_full_path = PathJoinSubstitution([
        get_package_share_directory('map_server'),
        'config',
        map_file_name
    ])

    # handle pgm loading accordingly using PythonExpression
    pgm_file_name = PathJoinSubstitution([
        get_package_share_directory('map_server'),
        'config',
        PythonExpression(["'", map_file_name, "'.replace('.yaml', '.pgm')"])
    ])

    return LaunchDescription([
        map_file_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename': map_file_full_path},
                        {'pgm_filename': pgm_file_name}],
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/user/ros2_ws/src/warehouse_project/localization_server/rviz/localization.rviz"],
            parameters=[{"use_sim_time": False}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])