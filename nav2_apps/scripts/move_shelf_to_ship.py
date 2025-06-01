#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node # Import Node to access create_client

# Import the custom service type
from custom_interfaces.srv import GoToLoading

# Import necessary classes for parameter setting
from rcl_interfaces.msg import Parameter # Note: Just Parameter, not ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import ParameterType # For specifying parameter type

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def set_node_parameter(node: Node, target_node_name: str, param_name: str, param_value, param_type: ParameterType):
    """
    Helper function to set a parameter on a target node.
    """
    param_client = node.create_client(SetParameters, f'{target_node_name}/set_parameters')
    
    node.get_logger().info(f'Waiting for parameter service for {target_node_name}...')
    # It's good practice to wait for the service, though it might already be up
    # if the target node is running. Add a timeout if desired.
    if not param_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'Parameter service for {target_node_name} not available. Skipping parameter set.')
        return False
    node.get_logger().info(f'Parameter service for {target_node_name} available.')

    request = SetParameters.Request()
    param = Parameter()
    param.name = param_name
    param.value.type = param_type

    if param_type == ParameterType.PARAMETER_STRING:
        param.value.string_value = str(param_value) # Ensure it's a string
    elif param_type == ParameterType.PARAMETER_DOUBLE:
        param.value.double_value = float(param_value) # Ensure it's a float
    # Add other types if needed (e.g., INTEGER, BOOL)
    else:
        node.get_logger().error(f"Unsupported parameter type: {param_type} for {param_name}")
        return False
        
    request.parameters.append(param)

    node.get_logger().info(f'Setting parameter "{param_name}" on "{target_node_name}" to {param_value}')
    future = param_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    success = False
    if future.result() is not None:
        for result_item in future.result().results:
            if result_item.successful:
                node.get_logger().info(f'Successfully set parameter "{param_name}" on "{target_node_name}"')
                success = True
            else:
                node.get_logger().error(f'Failed to set parameter "{param_name}" on "{target_node_name}": {result_item.reason}')
    else:
        node.get_logger().error(f'Parameter service call to "{target_node_name}" failed for "{param_name}".')
    return success

def main():
    ####################
    goal_pos = [5.3, 0.0]
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    approach_client = navigator.create_client(GoToLoading, '/approach_shelf')
    while not approach_client.wait_for_service(timeout_sec=1.0):
        navigator.get_logger().info('"/approach_shelf" service not available, waiting again...')
    navigator.get_logger().info('"/approach_shelf" service is available.')

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    cmd_vel_pub = navigator.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
    navigator.get_logger().info('Starting 360-degree rotation...')
    rotate_360_degrees(cmd_vel_pub, navigator) # Pass navigator for logging
    navigator.get_logger().info('Rotation complete!')

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = goal_pos[0]
    shelf_item_pose.pose.position.y = goal_pos[1]
    shelf_item_pose.pose.orientation.z = math.sin(-math.pi/4)
    shelf_item_pose.pose.orientation.w = math.cos(-math.pi/4)
    navigator.get_logger().info(f'Perceived a request to go to goal pos: {goal_pos}')
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            navigator.get_logger().info('Estimated time of arrival at goal position: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Got to goal pos!')

        navigator.get_logger().info('Calling "/approach_shelf" service...')
        request = GoToLoading.Request()
        request.attach_to_shelf = True
        future = approach_client.call_async(request)
        rclpy.spin_until_future_complete(navigator, future)

        service_call_successful_and_complete = False
        if future.result() is not None:
            response = future.result()
            if response.complete:
                navigator.get_logger().info('"/approach_shelf" service call succeeded and reported complete.')
                service_call_successful_and_complete = True
            else:
                 navigator.get_logger().warn('"/approach_shelf" service call succeeded but reported NOT complete.')
        else:
            navigator.get_logger().error('"/approach_shelf" service call failed %r' % future.exception())

        # --- Dynamically Change Footprint if service was successful and complete ---
        if service_call_successful_and_complete:
            navigator.get_logger().info('Attempting to update robot footprint parameters...')

            # --- LOCAL COSTMAP FOOTPRINT ---
            local_costmap_node_name = '/local_costmap/local_costmap' # Verified this worked for you
            local_footprint_param_name = 'footprint'
            new_local_footprint_string = "[[0.32, 0.32], [0.32, -0.32], [-0.32, -0.32], [-0.32, 0.32]]"
            
            set_node_parameter(navigator, 
                               local_costmap_node_name, 
                               local_footprint_param_name, 
                               new_local_footprint_string, 
                               ParameterType.PARAMETER_STRING)

            # --- GLOBAL COSTMAP RADIUS ---
            # !!! VERIFY THIS NODE NAME WITH 'ros2 node list' !!!
            # It might be '/global_costmap/global_costmap', '/planner_server', or similar
            global_costmap_node_name = '/global_costmap/global_costmap' 
            global_radius_param_name = 'robot_radius' # From your global_costmap config
            new_global_radius_value = 0.52 # Doubled from 0.15
            
            set_node_parameter(navigator,
                               global_costmap_node_name,
                               global_radius_param_name,
                               new_global_radius_value,
                               ParameterType.PARAMETER_DOUBLE)

            navigator.get_logger().info('Footprint parameter update process finished.')
            
            # Now you can proceed with the next navigation task, e.g., going to a shipping destination
            # navigator.get_logger().info('Proceeding to shipping destination with new footprint...')
            # shipping_dest = PoseStamped()
            # shipping_dest.header.frame_id = 'map'
            # shipping_dest.header.stamp = navigator.get_clock().now().to_msg()
            # shipping_dest.pose.position.x = NEW_X_COORDINATE
            # shipping_dest.pose.position.y = NEW_Y_COORDINATE
            # shipping_dest.pose.orientation.w = 1.0 # Or other orientation
            # navigator.goToPose(shipping_dest)
            # while not navigator.isTaskComplete():
            #     pass # Wait for this new task

        # --- End Dynamic Footprint Change ---

    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Task to goal pos was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Task to goal pos failed!')
        # Consider if you want to exit or try something else

    # This loop is needed to keep the script alive if a final task was given
    while not navigator.isTaskComplete():
        pass
    
    navigator.get_logger().info('Script finished.')
    # rclpy.shutdown() # BasicNavigator handles shutdown, or you can do it explicitly if needed
    exit(0)

def rotate_360_degrees(publisher, node: Node, angular_speed=0.5): # Added node for logging
    """
    Rotate the robot 360 degrees in place.
    """
    twist = Twist()
    twist.angular.z = angular_speed
    rotation_time = 2 * math.pi / abs(angular_speed)
    start_time = time.time()
    rate_sleep_time = 0.1

    while (time.time() - start_time) < rotation_time:
        if not rclpy.ok(): # Check if ROS is still running
            node.get_logger().warn("ROS shutting down, stopping rotation.")
            break
        publisher.publish(twist)
        time.sleep(rate_sleep_time)
    
    if rclpy.ok(): # Only publish stop if ROS is still okay
        twist.angular.z = 0.0
        publisher.publish(twist)
        time.sleep(0.5)

if __name__ == '__main__':
    main()
