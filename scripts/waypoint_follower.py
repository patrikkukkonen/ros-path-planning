#!/usr/bin/env python3

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

import signal
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import os

import math
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Quaternion

from rclpy.node import Node
from std_msgs.msg import Float32

# from distance import OdometryModifier
from distance2 import DistanceSubscriber

from rclpy.executors import MultiThreadedExecutor


# class CustomNavigator(BasicNavigator):
#     def __init__(self):
#         super().__init__()
#         self.start_time = None
#         self.total_distance = 0.0
#         self.previous_pose = None

#     def followWaypoints(self, waypoints):
#         self.start_time = time.time()
#         self.previous_pose = None
#         super().followWaypoints(waypoints)

#     def getFirstPathComputationTime(self):
#         return time.time() - self.start_time

#     def getDistanceTraveled(self):
#         return self.total_distance

#     def updateDistanceTraveled(self, current_pose):
#         if self.previous_pose is not None:
#             dx = current_pose.pose.position.x - self.previous_pose.pose.position.x
#             dy = current_pose.pose.position.y - self.previous_pose.pose.position.y
#             distance = (dx ** 2 + dy ** 2) ** 0.5
#             self.total_distance += distance

#         self.previous_pose = current_pose


"""
Basic navigation demo to go to poses.
"""   


def main():

    rclpy.init()

    navigator = BasicNavigator() # CustomNavigator() # BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 2.5
    # initial_pose.pose.position.y = -2.0
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses to follow
    goal_poses = []

    # PoseStamped() is a ROS2 message type for poses with a header (e.g. for frame_id and timestamp) and a pose (e.g. position and orientation)
    # Waypoint from initial point to first goal
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -2.5
    goal_pose1.pose.position.y = 2.5

    # Convert the desired yaw angle to a quaternion
    # yaw_angle = math.pi  # 180 degrees, facing west
    # quaternion = Quaternion()
    # quaternion.z = math.sin(yaw_angle / 2.0)
    # quaternion.w = math.cos(yaw_angle / 2.0)

    # goal_pose1.pose.orientation = quaternion
    # goal_poses.append(goal_pose1)

    # Orientation for first goal
    goal_pose1.pose.orientation.w = 0.5 # 1.0 #0.707
    goal_pose1.pose.orientation.z = 0.0 #-0.707
    goal_poses.append(goal_pose1)


    # Waypoint from the first goal back to the initial point
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.5
    goal_pose2.pose.position.y = -2.0

    # Convert the desired yaw angle to a quaternion
    # yaw_angle = math.pi / 2.0  # 90 degrees, facing north
    # quaternion = Quaternion()
    # quaternion.z = math.sin(yaw_angle / 2.0)
    # quaternion.w = math.cos(yaw_angle / 2.0)

    # goal_pose2.pose.orientation = quaternion
    # goal_poses.append(goal_pose2)

    # Orientation for second goal
    goal_pose2.pose.orientation.w = 0.5 #1.0 #0.707
    goal_pose2.pose.orientation.z = 0.0 #0.707
    goal_poses.append(goal_pose2)
    
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)


    # Pass in the goal poses to follow
    navigator.followWaypoints(goal_poses)
    
    #compute_time_end = time.time()
    #compute_time = compute_time_end - compute_time_start
    #print("Time to Compute: ", compute_time)


    nav_start = navigator.get_clock().now()
    start_time = time.time() # Track overall start time

    # send the goals to the navigator
    #compute_time_start = time.time()
    waypoint1_time = 0
    waypoint2_time = 0


    i = 0
    while not navigator.isTaskComplete():
        ##################################################
        #                                                #
        # Implement some code here for your application! #
        #                                                #
        ##################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()


            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()
            

            if feedback.current_waypoint == 1 and waypoint1_time == 0:
                waypoint_time = time.time()
                #print("Time to compute first path: ", waypoint_time - compute_time_start)
                waypoint1_time = waypoint_time - start_time
                # feedback.current_waypoint = 0
                #break
            
            # When second waypoint is being executed, start timer
            if feedback.current_waypoint == 1 and waypoint2_time == 0:
                waypoint_time = time.time()
                waypoint2_time = 1


            # Commenting this sovled the problem of the robot
            # stopping at midway and not being able to continue to 
            # Some follow waypoints request change to demo preemption
            # if now - nav_start > Duration(seconds=35.0):
            #     goal_pose4 = PoseStamped()
            #     goal_pose4.header.frame_id = 'map'
            #     goal_pose4.header.stamp = now.to_msg()
            #     goal_pose4.pose.position.x = -5.0
            #     goal_pose4.pose.position.y = -4.75
            #     goal_pose4.pose.orientation.w = 0.707
            #     goal_pose4.pose.orientation.z = 0.707
            #     goal_poses = [goal_pose4]
            #     nav_start = now
            #     navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')

        # Print out statistics if goal succeeded
        end_time = time.time() # Track overall end time
        total_time = end_time - start_time # Calculate total time
        
        # Print out statistics
        print("Waypoint 1 time: ", waypoint1_time)
        print("Waypoint 2 time: ", end_time - waypoint_time)

        print("Total time: ", total_time)

    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # Exit the program when the final goal is reached
    #os.killpg(os.getpgid(0), signal.SIGINT)  # Send termination signal to process group

    

if __name__ == '__main__':
    main()