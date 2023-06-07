#!/usr/bin/env python3
import rclpy
import tf2_ros
from math import sqrt
from nav_msgs.msg import Odometry

from std_msgs.msg import Float32


class OdometryModifier:

    def __init__(self):
        self.node = rclpy.create_node('odometry_modifier')
        self.sub = self.node.create_subscription(Odometry, 'odom', self.callback, 10)
        #self.pub = self.node.create_publisher(Odometry, 'odom2', 10)
        self.pub = self.node.create_publisher(Float32, 'distance_traveled', 10)
        self.total_distance = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.first_run = True
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def callback(self, data):
        if self.first_run:
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
            self.first_run = False

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        d_increment = sqrt((x - self.previous_x) * (x - self.previous_x) +
                           (y - self.previous_y) * (y - self.previous_y))
        self.total_distance += d_increment
        print("Total distance traveled is {:.2f}m".format(self.total_distance))

        #self.pub.publish(data)
        #self.pub.publish(self.total_distance)

        #distance_pub = self.total_distance

        #self.pub.publish(distance_pub)

        # Create a Float32 message instance
        # message = Float32()
        # value = 14.804075241088867
        # formatted_value = "{:.2f}".format(value)  # Format the float value to two decimal places
        # message.data = float(formatted_value)  # Assign the formatted value to the message


        message = Float32()
        message.data = self.total_distance
        self.pub.publish(message)

        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y
    

    #def get_total_distance(self):
    #    return self.total_distance


    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    odom = OdometryModifier()
    rclpy.spin(odom.node)#(odom.node)  # Pass the node instance instead of the OdometryModifier instance
    # After the node is destroyed, get the total distance
    #final_distance = odom.get_total_distance()
    #print("Final distance traveled is {:.2f}m".format(final_distance))
    odom.destroy()
    #rclpy.shutdown()
    # After the node is destroyed, get the total distance
    #final_distance = odom.get_total_distance()
    #print("Final distance traveled is {:.2f}m".format(final_distance))


if __name__ == '__main__':
    main()
