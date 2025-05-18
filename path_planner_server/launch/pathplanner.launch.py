import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition # Import conditional classes

def generate_launch_description():
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False', # Keep a default value
        description='Use simulation clock')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define conditions based on the use_sim_time LaunchConfiguration
    # Compare to the string 'true' (lowercase) as passed by the launch system
    is_sim = IfCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))
    is_real = UnlessCondition(PythonExpression(["'", use_sim_time, "' == 'true'"]))

    config_dir = get_package_share_directory('path_planner_server')

    # Determine config file names based on the condition, using PythonExpression for the filename part
    controller_config_name = PythonExpression([
        "'controller_sim.yaml' if '", use_sim_time, "' == 'true' else 'controller_real.yaml'"
    ])
    bt_navigator_config_name = PythonExpression([
        "'bt_navigator_sim.yaml' if '", use_sim_time, "' == 'true' else 'bt_navigator_real.yaml'"
    ])
    planner_config_name = PythonExpression([
        "'planner_sim.yaml' if '", use_sim_time, "' == 'true' else 'planner_real.yaml'"
    ])
    recovery_config_name = PythonExpression([
        "'recoveries_sim.yaml' if '", use_sim_time, "' == 'true' else 'recoveries_real.yaml'"
    ])

    # Construct full paths using PathJoinSubstitution
    controller_yaml = PathJoinSubstitution([config_dir, 'config', controller_config_name])
    bt_navigator_yaml = PathJoinSubstitution([config_dir, 'config', bt_navigator_config_name])
    planner_yaml = PathJoinSubstitution([config_dir, 'config', planner_config_name])
    recovery_yaml = PathJoinSubstitution([config_dir, 'config', recovery_config_name])

    # Define remappings list for the simulation case
    cmd_vel_remappings_sim = [('cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]

    return LaunchDescription([
        use_sim_time_param, # Use use_sim_time_param as declared

        # Node for controller_server in simulation (with remapping)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=cmd_vel_remappings_sim, # Apply remapping
            condition=is_sim # Only launch this node in simulation
        ),

        # Node for controller_server on real robot (without remapping)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[], # No remapping
            condition=is_real # Only launch this node on the real robot
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
                                        'controller_server', # Note: We use 'controller_server' name for lifecycle management, even though there are two conditional Node actions using this name. The lifecycle manager will target the one that is actually launched.
                                        'recoveries_server',
                                        'bt_navigator']}]
        )
    ])
