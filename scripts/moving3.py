#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyLinkWrench
from geometry_msgs.msg import Wrench

class BoxMover(Node):
    def __init__(self):
        super().__init__('box_mover')
        self.apply_wrench = self.create_client(ApplyLinkWrench, '/gazebo/apply_link_wrench')
        while not self.apply_wrench.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def move_box(self):
        # Apply a wrench (force and torque) to the box model to make it move
        wrench = Wrench()
        wrench.force.x = 10.0  # set the force in x direction to 10 N
        wrench.torque.z = 1.0  # set the torque around z axis to 1 Nm
        request = ApplyLinkWrench.Request()
        request.body_name = 'obstacle::link'  # the name of your box model in Gazebo
        request.wrench = wrench
        request.reference_frame = 'world'
        self.apply_wrench.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    box_mover = BoxMover()
    box_mover.move_box()
    rclpy.spin_once(box_mover, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
