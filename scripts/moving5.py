#!/usr/bin/env python3

import rclpy
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState

from geometry_msgs.msg import Twist
from time import sleep

class Moving:
    def __init__(self):
        super(Moving, self).__init__()

        self.node = rclpy.create_node('moving_obstacle')
        self.client_spawn = self.node.create_client(SpawnEntity, '/spawn_entity')
        self.client_get_state = self.node.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.client_set_state = self.node.create_client(SetEntityState, '/gazebo/set_entity_state')
        

        while not self.client_spawn.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('SpawnEntity service not available, waiting again...')

        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('GetEntityState service not available, waiting again...')

        while not self.client_set_state.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('SetEntityState service not available, waiting again...')

        self.vel_pub = self.node.create_publisher(Twist, '/obstacle/pose', 10)
        self.move_box()

    def move_box(self):
        # Spawn a new model in Gazebo
        model_xml = """
        <!-- An actor -->
      <actor name="obstacle">
        <link name="link">
          <visual name="visual">
            <geometry>
              <box>
                <size>.2 .2 .2</size>
              </box>
            </geometry>
          </visual>
        </link>
              <script>
          <loop>true</loop>
          <delay_start>0.000000</delay_start>
          <auto_start>true</auto_start>
             <trajectory id="0" type="square">
             <waypoint>
                <time>0.0</time>
                <pose>-1 -1 1 0 0 0</pose>
             </waypoint>
             <waypoint>
                <time>1.0</time>
                <pose>-1 1 1 0 0 0</pose>
             </waypoint>
             <waypoint>
                <time>2.0</time>
                <pose>1 1 1 0 0 0</pose>
             </waypoint>
             <waypoint>
                <time>3.0</time>
                <pose>1 -1 1 0 0 0</pose>
             </waypoint>
             <waypoint>
                <time>4.0</time>
                <pose>-1 -1 1 0 0 0</pose>
             </waypoint>
          </trajectory>
        </script>
      </actor>
            """
        spawn_req = SpawnEntity.Request()
        spawn_req.name = 'obstacle'
        spawn_req.xml = model_xml
        self.client_spawn.call_async(spawn_req)

        # Move the model back and forth between two points
        rclpy.spin_once(self.node, timeout_sec=1)
        vel = Twist()
        #vel.linear.x = 5.0  # set the linear speed to 0.5 m/s
        vel.linear.y = 5.0
        #x_pos = 0.0  # set initial x position to 0.0
        #pose = Pose()
        #pose.position.x = 0.0
        direction = 1  # set the direction to forward

        while rclpy.ok():
            self.vel_pub.publish(vel)  # publish linear velocity to cmd_vel topic
            self.node.get_logger().info('Moving obstacle')

            # check if the box has reached the end of its travel in the current direction
            if direction == 1 and self.get_box_position().x > 2.0:
                direction = -1  # switch direction of travel
            elif direction == -1 and self.get_box_position().x < -2.0:
                direction = 1  # switch direction of travel

            # reverse the direction of travel if necessary
            vel.linear.x = 1.0 * direction
            rclpy.spin_once(self.node, timeout_sec=0.1)

            print("X-pos: ", self.get_box_position().x)

    def get_box_position(self):
        # create a client for the GetEntityState service
        get_entity_state_client = self.node.create_client(GetEntityState, '/gazebo/get_entity_state')
        # create the request message
        request = GetEntityState.Request()
        request.name = 'obstacle' # replace 'box' with your entity name
        
        while not get_entity_state_client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        
        # send the request to the service and wait for the response
        future = get_entity_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        
        # check if the service call was successful and return the box position
        if future.result() is not None:
            return future.result().state.pose.position
        else:
            self.node.get_logger().warn('Failed to get box position')
            return None

def main():
    rclpy.init()
    moving = Moving()
    rclpy.spin(moving.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()