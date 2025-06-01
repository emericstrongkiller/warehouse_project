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
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import ParameterType

# Import for clearing costmaps # REMOVED: from std_srvs.srv import Empty


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def set_node_parameter(node: Node, target_node_name: str, param_name: str, param_value, param_type: ParameterType):
    """
    Helper function to set a parameter on a target node.
    """
    param_client = node.create_client(SetParameters, f'{target_node_name}/set_parameters')
    node.get_logger().info(f'Waiting for parameter service for {target_node_name}...')
    if not param_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'Parameter service for {target_node_name} not available. Skipping parameter set.')
        return False
    node.get_logger().info(f'Parameter service for {target_node_name} available.')

    request = SetParameters.Request()
    param = Parameter()
    param.name = param_name
    param.value.type = param_type

    if param_type == ParameterType.PARAMETER_STRING:
        param.value.string_value = str(param_value)
    elif param_type == ParameterType.PARAMETER_DOUBLE:
        param.value.double_value = float(param_value)
    else:
        node.get_logger().error(f"Unsupported parameter type: {param_type} for {param_name}")
        return False
    request.parameters.append(param)

    node.get_logger().info(f'Setting parameter "{param_name}" on "{target_node_name}" to {param_value}')
    future = param_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0) # Added timeout

    success = False
    if future.result() is not None:
        for result_item in future.result().results:
            if result_item.successful:
                node.get_logger().info(f'Successfully set parameter "{param_name}" on "{target_node_name}"')
                success = True
            else:
                node.get_logger().error(f'Failed to set parameter "{param_name}" on "{target_node_name}": {result_item.reason}')
    else:
        node.get_logger().error(f'Parameter service call to "{target_node_name}" failed for "{param_name}" (timed out or other error).')
    return success

# REMOVED clear_costmap function

def move_robot_timed(publisher: rclpy.publisher.Publisher, node: Node, linear_x: float, angular_z: float, duration_sec: float):
    """
    Moves the robot by publishing to cmd_vel for a specific duration.
    """
    node.get_logger().info(f"Moving robot: linear_x={linear_x}, angular_z={angular_z} for {duration_sec}s")
    twist_msg = Twist()
    twist_msg.linear.x = linear_x
    twist_msg.angular.z = angular_z

    start_time = time.time()
    rate_sleep_time = 0.1 # Publish at 10Hz

    while (time.time() - start_time) < duration_sec:
        if not rclpy.ok():
            node.get_logger().warn("ROS shutting down, stopping timed movement.")
            break
        publisher.publish(twist_msg)
        time.sleep(rate_sleep_time)

    # Stop the robot
    if rclpy.ok():
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        publisher.publish(twist_msg)
        node.get_logger().info("Timed movement finished, robot stopped.")
        time.sleep(0.5) # Give a moment for the stop command to take effect

def main():
    ####################
    goal_pos = [5.3, 0.0]
    shipping_pos = [2.1, 1.3]
    backward_move_duration = 7.0  # seconds
    backward_move_speed = -0.2    # m/s (negative for backward)
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
    rotate_360_degrees(cmd_vel_pub, navigator)
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
        i = (i + 1) % 100 # Prevent i from growing indefinitely
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            navigator.get_logger().info('ETA at goal: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + 's.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Got to goal pos (under shelf)!')

        navigator.get_logger().info('Calling "/approach_shelf" service to lift shelf...')
        request = GoToLoading.Request()
        request.attach_to_shelf = True
        future = approach_client.call_async(request)
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=10.0) # Added timeout

        service_call_successful_and_complete = False
        if future.done() and future.result() is not None:
            response = future.result()
            if response.complete:
                navigator.get_logger().info('"/approach_shelf" service call succeeded and reported complete.')
                service_call_successful_and_complete = True
            else:
                 navigator.get_logger().warn('"/approach_shelf" service call succeeded but reported NOT complete.')
        else:
            if future.done() and future.exception() is not None:
                navigator.get_logger().error(f'"/approach_shelf" service call failed: {future.exception()}')
            else:
                navigator.get_logger().error('"/approach_shelf" service call timed out or failed without exception.')


        if service_call_successful_and_complete:
            navigator.get_logger().info('Attempting to update robot footprint parameters...')
            local_costmap_node_name = '/local_costmap/local_costmap'
            local_footprint_param_name = 'footprint'
            # Using the larger footprint now
            new_local_footprint_string = "[[0.40, 0.40], [0.40, -0.40], [-0.40, -0.40], [-0.40, 0.40]]"
            set_node_parameter(navigator, local_costmap_node_name, local_footprint_param_name, new_local_footprint_string, ParameterType.PARAMETER_STRING)

            # !!! VERIFY THIS NODE NAME WITH 'ros2 node list' !!!
            global_costmap_node_name = '/global_costmap/global_costmap'
            global_radius_param_name = 'robot_radius'
            new_global_radius_value = 0.40 # Larger radius
            set_node_parameter(navigator, global_costmap_node_name, global_radius_param_name, new_global_radius_value, ParameterType.PARAMETER_DOUBLE)
            navigator.get_logger().info('Footprint parameter update process finished.')

            # --- Clear Costmaps --- REMOVED
            # navigator.get_logger().info("Attempting to clear costmaps after footprint update...")
            # Verify these service names with `ros2 service list | grep clear`
            # clear_costmap(navigator, "local_costmap", "clear_entirely_local_costmap")
            # clear_costmap(navigator, "global_costmap", "clear_entirely_global_costmap")
            # navigator.get_logger().info("Costmap clearing process finished.")
            # time.sleep(1.0) # Short pause after clearing

            # --- MOVE BACKWARDS ---
            move_robot_timed(cmd_vel_pub, navigator, backward_move_speed, 0.0, backward_move_duration)
            # --- END MOVE BACKWARDS ---
            
            time.sleep(1.0) # Short pause before next navigation

            # --- NAVIGATE TO SHIPPING DESTINATION ---
            navigator.get_logger().info(f'Proceeding to shipping destination at {shipping_pos}...')
            shipping_dest_pose = PoseStamped()
            shipping_dest_pose.header.frame_id = 'map'
            shipping_dest_pose.header.stamp = navigator.get_clock().now().to_msg()
            shipping_dest_pose.pose.position.x = shipping_pos[0]
            shipping_dest_pose.pose.position.y = shipping_pos[1]
            # Set an appropriate orientation for the shipping destination
            # For example, facing a certain direction or just default (w=1.0)
            q = tf_transformations.quaternion_from_euler(0, 0, 0) # Example: no rotation from map's X-axis
            shipping_dest_pose.pose.orientation.x = q[0]
            shipping_dest_pose.pose.orientation.y = q[1]
            shipping_dest_pose.pose.orientation.z = q[2]
            shipping_dest_pose.pose.orientation.w = q[3]
            
            navigator.goToPose(shipping_dest_pose)

            i = 0 # Reset feedback counter
            while not navigator.isTaskComplete():
                i = (i + 1) % 100
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    navigator.get_logger().info('ETA at shipping dest: ' + '{0:.0f}'.format(
                              Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + 's.')

            shipping_result = navigator.getResult()
            if shipping_result == TaskResult.SUCCEEDED:
                navigator.get_logger().info('Successfully reached shipping destination!')
            elif shipping_result == TaskResult.CANCELED:
                navigator.get_logger().warn('Navigation to shipping destination was canceled.')
            elif shipping_result == TaskResult.FAILED:
                navigator.get_logger().error('Navigation to shipping destination FAILED!')
            # --- END NAVIGATE TO SHIPPING DESTINATION ---

    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Task to goal pos was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete(): pass # Wait for return

    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Task to goal pos FAILED!')

    navigator.get_logger().info('Script finished.')
    exit(0)

def rotate_360_degrees(publisher, node: Node, angular_speed=0.5):
    twist = Twist()
    twist.angular.z = angular_speed
    rotation_time = 2 * math.pi / abs(angular_speed)
    start_time = time.time()
    rate_sleep_time = 0.1
    while (time.time() - start_time) < rotation_time:
        if not rclpy.ok():
            node.get_logger().warn("ROS shutting down, stopping rotation.")
            break
        publisher.publish(twist)
        time.sleep(rate_sleep_time)
    if rclpy.ok():
        twist.angular.z = 0.0
        publisher.publish(twist)
        time.sleep(0.5)

# You'll need tf_transformations for quaternion_from_euler
# If not already installed: pip install tf-transformations
import tf_transformations

if __name__ == '__main__':
    main()