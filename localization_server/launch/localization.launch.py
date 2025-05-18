import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():

    # Declare the launch argument for the map file name
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        description='Name of the map file (without the path) to be launched')

    # Get the value of the launch argument as a LaunchConfiguration
    map_file_name = LaunchConfiguration('map_file')

    # Construct the full path to the map file using PathJoinSubstitution
    map_server_config_dir = get_package_share_directory('map_server')
    map_file_full_path = PathJoinSubstitution([
        map_server_config_dir,
        'config',
        map_file_name
    ])

    # Handle pgm loading accordingly
    pgm_file_name = PathJoinSubstitution([
        map_server_config_dir,
        'config',
        PythonExpression(["'", map_file_name, "'.replace('.yaml', '.pgm')"])
    ])

    # Determine simulation mode based on map name (simple substring check)
    # This expression evaluates to a Python boolean (True or False)
    is_sim = PythonExpression(["'sim' in '", map_file_name, "'"])

    # Construct the full path to the AMCL config file based on simulation mode
    localization_server_config_dir = get_package_share_directory('localization_server')
    amcl_config_file = PathJoinSubstitution([
        localization_server_config_dir,
        'config',
        PythonExpression([
            "'amcl_config_sim.yaml' if ", is_sim, " else 'amcl_config_real.yaml'"
        ])
    ])


    return LaunchDescription([
        map_file_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': is_sim},
                {'yaml_filename': map_file_full_path},
                {'pgm_filename': pgm_file_name}
            ],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_file, {'use_sim_time': is_sim}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/user/ros2_ws/src/warehouse_project/localization_server/rviz/localization.rviz"],
            parameters=[{"use_sim_time": is_sim}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': is_sim},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        )
    ])
