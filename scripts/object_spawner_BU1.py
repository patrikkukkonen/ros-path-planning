#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')

        # Create a client to call the Gazebo spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Create a client to call the Gazebo delete service
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

    def spawn_object(self, name, model, pose):
        """Spawn an object in Gazebo"""
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo spawn service not available, waiting...')

        request = SpawnEntity.Request()
        request.name = name
        request.xml = model
        request.initial_pose = pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Object spawned successfully: %s' % name)
        else:
            self.get_logger().error('Failed to spawn object: %s' % name)

    def delete_object(self, name):
        """Delete an object from Gazebo"""
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo delete service not available, waiting...')

        request = DeleteEntity.Request()
        request.name = name

        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Object deleted successfully: {}'.format(name))
        else:
            self.get_logger().error('Failed to delete object: {}'.format(name))

def main():
    rclpy.init()
    spawner = ObjectSpawner()

    # Example usage
    obstacle_name = 'obstacle'
    obstacle_model = """
        <?xml version="1.0"?>
        <sdf version="1.7">
        <model name="obstacle">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>0.1 0.1 0.25</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.1 0.1 0.25</size>
                        </box>
                    </geometry>
                </visual>
            </link>
        </model>
        </sdf>
    """

    # Function to spawn the obstacle at a specific pose
    def spawn_objects(id, num_objects, spawn_interval, x_pose, y_pose, z_pose, horizontal=True):
        """Spawn multiple objects in Gazebo"""
        if horizontal:
            for i in range(num_objects):
                # Spawn the obstacle at a specific poses
                obstacle_name = f'obstacle{id}_{i}'
                obstacle_pose = Pose()
                obstacle_pose.position.x = x_pose #-1.0
                obstacle_pose.position.y = y_pose + i * -0.15 #-0.1 + i*-0.15
                obstacle_pose.position.z = z_pose #0.5
                spawner.spawn_object(obstacle_name, obstacle_model, obstacle_pose)

                time.sleep(spawn_interval)
        else: # if vertical
            for i in range(num_objects):
                # Spawn the obstacle at a specific poses
                obstacle_name = f'obstacle{id}_{i}'
                obstacle_pose = Pose()
                obstacle_pose.position.x = x_pose + i * 0.15 #-1.0
                obstacle_pose.position.y = y_pose
                obstacle_pose.position.z = z_pose
                spawner.spawn_object(obstacle_name, obstacle_model, obstacle_pose)

                time.sleep(spawn_interval)

    # Spawn the obstacle at a specific pose
    # obstacle_pose = Pose()
    # obstacle_pose.position.x = 1.0
    # obstacle_pose.position.y = 2.0
    # obstacle_pose.position.z = 0.5
    # spawner.spawn_object(obstacle_name, obstacle_model, obstacle_pose)

    # # Wait for some time before deleting the obstacle
    # #rclpy.spin_once(spawner, timeout_sec=5.0)
    # time.sleep(5.0)

    # # Delete the obstacle
    # spawner.delete_object(obstacle_name)

    # loop
    num_objects = 6
    spawn_interval = 0.5

    for i in range(num_objects):
        # Spawn the obstacle at a specific poses
        obstacle_name = f'obstacle_{i}'
        obstacle_pose = Pose()
        obstacle_pose.position.x = -1.0
        obstacle_pose.position.y = -0.1 + i*-0.15
        obstacle_pose.position.z = 0.5
        spawner.spawn_object(obstacle_name, obstacle_model, obstacle_pose)

        time.sleep(spawn_interval)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
