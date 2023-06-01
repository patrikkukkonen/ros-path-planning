#!/usr/bin/env python3

from object_spawner import spawn_objects
import rclpy

def main():

    rclpy.init()
    # Create an instance of the ObjectSpawner class
    spawner = ObjectSpawner()
    # spawn_obejcts(id, num_objects, spawn_interval, x_pose, y_pose, z_pose, horizontal=True)
        # Call the function to spawn the obstacles to block the robot path
    spawn_objects(id=1, num_objects=6, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.1, z_pose=0.5, horizontal=True)    
    spawn_objects(id=2, num_objects=6, spawn_interval=0.5, x_pose=-1.0, y_pose=-2.1, z_pose=0.5, horizontal=True)    
    spawn_objects(id=3, num_objects=6, spawn_interval=0.5, x_pose=1.1, y_pose=2.0, z_pose=0.5, horizontal=False)
    spawn_objects(id=4, num_objects=6, spawn_interval=0.5, x_pose=2.0, y_pose=-1.1, z_pose=0.5, horizontal=True)
    rclpy.shutdown()

if __name__ == '__main__':
    main()