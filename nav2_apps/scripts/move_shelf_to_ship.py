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


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    goal_pos = [5.3, 0.0]
    ####################

    rclpy.init()

    # BasicNavigator inherits from rclpy.node.Node, so we can use its methods
    navigator = BasicNavigator()

    # --- Service Client Setup ---
    # Create the service client for the approach_shelf service
    approach_client = navigator.create_client(GoToLoading, '/approach_shelf')

    # Wait for the service server to be available
    while not approach_client.wait_for_service(timeout_sec=1.0):
        navigator.get_logger().info('"/approach_shelf" service not available, waiting again...')
    navigator.get_logger().info('"/approach_shelf" service is available.')
    # --- End Service Client Setup ---


    # Set your demo's initial pose
    # get robot_base_footprint xy positions relative to map frame
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    # Using valid quaternion values (this is identity rotation - no rotation)
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # First, perform a 360-degree rotation using cmd_vel
    cmd_vel_pub = navigator.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
    print('Starting 360-degree rotation...')
    rotate_360_degrees(cmd_vel_pub)
    print('Rotation complete!')

    # Now, proceed with navigation
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = goal_pos[0]
    shelf_item_pose.pose.position.y = goal_pos[1]
    # Using valid quaternion values
    shelf_item_pose.pose.orientation.x = 0.0
    shelf_item_pose.pose.orientation.y = 0.0
    shelf_item_pose.pose.orientation.z = math.sin(-math.pi/4)
    shelf_item_pose.pose.orientation.w = math.cos(-math.pi/4)
    print('Perceived a request to go to goal pos')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at goal position: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got to goal pos !')

        # --- Call the service after successful navigation ---
        navigator.get_logger().info('Calling "/approach_shelf" service...')
        request = GoToLoading.Request()
        request.attach_to_shelf = True # Set the service request field
        future = approach_client.call_async(request) # Call the service asynchronously

        # Wait for the service call to complete
        rclpy.spin_until_future_complete(navigator, future)

        if future.result() is not None:
            response = future.result()
            if response.complete:
                navigator.get_logger().info('"/approach_shelf" service call succeeded and reported complete.')
            else:
                 navigator.get_logger().info('"/approach_shelf" service call succeeded but reported not complete.')
        else:
            navigator.get_logger().error('"/approach_shelf" service call failed %r' % future.exception())
        # --- End service call ---

    elif result == TaskResult.CANCELED:
        print('Task to goal pos was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task to goal pos failed!')
        exit(-1)

    # This loop is needed to keep the script alive if the task failed
    # and it needed to go back to initial pose. It's harmless otherwise.
    while not navigator.isTaskComplete():
        pass

    exit(0)

def rotate_360_degrees(publisher, angular_speed=0.5):
    """
    Rotate the robot 360 degrees in place.

    Args:
        publisher: The cmd_vel publisher
        angular_speed: Angular speed in radians/second (positive for counterclockwise)
    """
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_speed

    # Calculate time needed for a full 360-degree rotation
    # 2π radians = 360 degrees
    rotation_time = 2 * math.pi / abs(angular_speed) # Corrected rotation time calculation

    # Start rotation
    start_time = time.time()

    # Publish cmd_vel messages until we complete the rotation
    rate = 0.1  # 10 Hz
    while time.time() - start_time < rotation_time:
        publisher.publish(twist)
        time.sleep(rate)

    # Stop rotation
    twist.angular.z = 0.0
    publisher.publish(twist)
    time.sleep(0.5)  # Short delay to ensure the robot has stopped

if __name__ == '__main__':
    main()
