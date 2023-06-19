#!/usr/bin/env python3

import time
from object_spawner import ObjectSpawner
import rclpy

def main():

    rclpy.init()
    # Create an instance of the ObjectSpawner class
    spawner = ObjectSpawner()
    # spawn_obejcts(id, num_objects, spawn_interval, x_pose, y_pose, z_pose, horizontal=True)
        # Call the function to spawn the obstacles to block the robot path
    
    # spawner.spawn_objects(id=1, num_objects=6, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.1, z_pose=0.5, horizontal=True)    
    # spawner.spawn_objects(id=2, num_objects=6, spawn_interval=0.5, x_pose=-1.0, y_pose=-2.1, z_pose=0.5, horizontal=True)    
    # spawner.spawn_objects(id=3, num_objects=6, spawn_interval=0.5, x_pose=1.1, y_pose=2.0, z_pose=0.5, horizontal=False)
    # spawner.spawn_objects(id=4, num_objects=6, spawn_interval=0.5, x_pose=0.1, y_pose=0.0, z_pose=0.5, horizontal=False)
    # reduced objects to improve performance
    # spawner.spawn_objects(id=1, num_objects=2, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.25, z_pose=0.5, horizontal=True)
    # spawner.spawn_objects(id=2, num_objects=2, spawn_interval=0.5, x_pose=-1.0, y_pose=-2.25, z_pose=0.5, horizontal=True)
    # spawner.spawn_objects(id=3, num_objects=2, spawn_interval=0.5, x_pose=1.25, y_pose=2.0, z_pose=0.5, horizontal=False)
    # spawner.spawn_objects(id=4, num_objects=2, spawn_interval=0.5, x_pose=0.25, y_pose=0.0, z_pose=0.5, horizontal=False)
    # time.sleep(15)
    # # delete obstacles
    # spawner.delete_objects(id=1, num_objects=6, delete_interval=0.5)
    # spawner.delete_objects(id=2, num_objects=6, delete_interval=0.5)
    # spawner.delete_objects(id=3, num_objects=6, delete_interval=0.5)

    # # spawner.spawn_objects(id=5, num_objects=6, spawn_interval=0.5, x_pose=2.0, y_pose=-1.1, z_pose=0.5, horizontal=True)
    # # spawner.spawn_objects(id=6, num_objects=6, spawn_interval=0.5, x_pose=0.0, y_pose=-2.1, z_pose=0.5, horizontal=True)
    # # reduced objects to improve performance
    # spawner.spawn_objects(id=5, num_objects=2, spawn_interval=0.5, x_pose=2.0, y_pose=-1.25, z_pose=0.5, horizontal=True)
    # spawner.spawn_objects(id=6, num_objects=2, spawn_interval=0.5, x_pose=0.0, y_pose=-2.25, z_pose=0.5, horizontal=True)

    # # delete obstacles
    # time.sleep(20)
    # spawner.delete_objects(id=6, num_objects=6, delete_interval=0.5)

    # reduce even more
    #time.sleep(10)
    for i in range(10):
        spawner.spawn_objects(id=1, num_objects=1, spawn_interval=0.5, x_pose=-1.0, y_pose=-0.5, z_pose=0.5, horizontal=True)
        time.sleep(10)
        spawner.spawn_objects(id=2, num_objects=1, spawn_interval=0.5, x_pose=-1.0, y_pose=-2.5, z_pose=0.5, horizontal=True)
        time.sleep(10)
        spawner.spawn_objects(id=3, num_objects=1, spawn_interval=0.5, x_pose=1.0, y_pose=1.5, z_pose=0.5, horizontal=True)
        time.sleep(10)
        spawner.spawn_objects(id=4, num_objects=1, spawn_interval=0.5, x_pose=0.5, y_pose=0.0, z_pose=0.5, horizontal=False)
        
        spawner.delete_objects(id=2, num_objects=1, delete_interval=0.5) # 2
        time.sleep(10)
        spawner.delete_objects(id=1, num_objects=1, delete_interval=0.5) # 1
        #time.sleep(10)
        #spawner.delete_objects(id=3, num_objects=1, delete_interval=0.5) # remove
        #time.sleep(10)
        #spawner.delete_objects(id=4, num_objects=1, delete_interval=0.5) # remove 
        #time.sleep(10)
        spawner.spawn_objects(id=5, num_objects=1, spawn_interval=0.5, x_pose=2.0, y_pose=-1.5, z_pose=0.5, horizontal=True)
        spawner.delete_objects(id=6, num_objects=1, delete_interval=0.5) # remove
        time.sleep(10)
        spawner.delete_objects(id=5, num_objects=1, delete_interval=0.5)
        # time.sleep(10)
        #time.sleep(10)
        spawner.spawn_objects(id=6, num_objects=1, spawn_interval=0.5, x_pose=0.0, y_pose=-2.5, z_pose=0.5, horizontal=True)
        time.sleep(10)

        #spawner.delete_objects(id=3, num_objects=1, delete_interval=0.5) # remove
        #time.sleep(10)

        spawner.delete_objects(id=4, num_objects=1, delete_interval=0.5) # remove
        time.sleep(10)


    rclpy.shutdown()

if __name__ == '__main__':
    main()