#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist

class SlidingBox(Node):
    def __init__(self):
        super().__init__('sliding_box')
        self.box_state = SetEntityState.Request()
        self.box_state.state.twist.linear.x = 1.0
        self.box_state.state.pose.position.z = 0.5
        self.box_state.state.pose.orientation.w = 1.0
        self.box_state.state.name = 'obstacle'
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_entity_state service not available, waiting again...')

        self.timer = self.create_timer(0.01, self.update_box_position)

    def update_box_position(self):
        try:
            future = self.set_entity_state_client.call_async(self.box_state)
            rclpy.spin_until_future_complete(self, future)
        except Exception as e:
            self.get_logger().error(f'Service call failed {str(e)}')
            return
        box_pose = future.result().success
        if box_pose:
            x = box_pose.pose.position.x
            if x >= 2.0:
                self.box_state.state.twist.linear.x = -1.0
            elif x <= -2.0:
                self.box_state.state.twist.linear.x = 1.0

def main(args=None):
    rclpy.init(args=args)
    sliding_box = SlidingBox()
    rclpy.spin(sliding_box)
    sliding_box.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()