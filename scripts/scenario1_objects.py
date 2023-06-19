#!/usr/bin/env python3

from object_spawner import ObjectSpawner
import rclpy
import time

def main():
    rclpy.init()
    # Create an instance of the ObjectSpawner class
    spawner = ObjectSpawner()
    # spawn_obejcts(id, num_objects, spawn_interval, x_pose, y_pose, z_pose, horizontal=True)
    #spawner.spawn_objects(id=1, num_objects=6, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.1, z_pose=0.5, horizontal=True)
    # reduced objects to improve performance
    # spawner.spawn_objects(id=1, num_objects=2, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.25, z_pose=0.5, horizontal=True)
    
    # 
    # reduce even more
    spawner.spawn_objects(id=1, num_objects=1, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.5, z_pose=0.5, horizontal=True)

    #spawner.delete_objects(id=1, num_objects=6, delete_interval=0.5)
    rclpy.shutdown()
if __name__ == '__main__':
    main()