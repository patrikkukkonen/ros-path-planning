#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DistanceSubscriber(Node):

    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription_ = self.create_subscription(
            Float32,
            'distance_traveled',
            self.distance_callback,
            10
        )
        self.received_distance_ = None


    def distance_callback(self, msg):
        self.received_distance_ = msg.data
        self.get_logger().info('Received: {}'.format(msg.data))


    def get_received_distance(self):
        print("GET RECEIVED DISTANCE")
        print(self.received_distance_)
        return self.received_distance_


def main(args=None):
    rclpy.init(args=args)
    subscriber = DistanceSubscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()