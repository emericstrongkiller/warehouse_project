import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='use simulation clock')

    use_sim_time = LaunchConfiguration('use_sim_time')

    config_dir = get_package_share_directory('path_planner_server')

    controller_yaml = PathJoinSubstitution([
        config_dir,
        'config',
        PythonExpression([
            "'controller_sim.yaml' if ", use_sim_time, " == 'True' else 'controller_real.yaml'"
        ])
    ])

    bt_navigator_yaml = PathJoinSubstitution([
        config_dir,
        'config',
        PythonExpression([
            "'bt_navigator_sim.yaml' if ", use_sim_time, " == 'True' else 'bt_navigator_real.yaml'"
        ])
    ])

    planner_yaml = PathJoinSubstitution([
        config_dir,
        'config',
        PythonExpression([
            "'planner_sim.yaml' if ", use_sim_time, " == 'True' else 'planner_real.yaml'"
        ])
    ])

    recovery_yaml = PathJoinSubstitution([
        config_dir,
        'config',
        PythonExpression([
            "'recoveries_sim.yaml' if ", use_sim_time, " == 'True' else 'recoveries_real.yaml'"
        ])
    ])

    return LaunchDescription([
        use_sim_time_cmd,

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/path_planner.rviz"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]
        )
    ])