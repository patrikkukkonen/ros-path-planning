#import rospy
import rclpy
from rclpy.node import Node

import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates

# TODO: MAKE THE OBSTACLES MOVE

class Moving(Node):
    def __init__(self):
        super().__init__('moving_obstacle')
        self.pub_model = self.create_publisher(ModelState, 'gazebo/set_model_state', 1)
        self.sub_model = self.create_subscription(ModelStates, 'gazebo/model_states', self.callback_model, 1)
        #self.sub_model  # prevent unused variable warning
        self.moving()
    def callback_model(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == 'unit_box':
                obstacle = ModelState()
                obstacle.model_name = 'unit_box'
                obstacle.pose = msg.pose[i]
                obstacle.twist = Twist()
                obstacle.twist.angular.z = 0.5
                self.pub_model.publish(obstacle)

    def moving(self):
        while rclpy.ok():
            rclpy.spin_once(self)
    # def moving(self):
    #     while rclpy.ok():
    #         obstacle = ModelState()
    #         model = self.wait_for_message('gazebo/model_states', ModelStates)
    #         for i in range(len(model.name)):
    #             if model.name[i] == 'obstacle':
    #                 obstacle.model_name = 'obstacle'
    #                 obstacle.pose = model.pose[i]
    #                 obstacle.twist = Twist()
    #                 obstacle.twist.angular.z = 0.5
    #                 self.pub_model.publish(obstacle)
    #                 time.sleep(0.1)
        # while not rospy.is_shutdown():
        #     obstacle = ModelState()
        #     model = rospy.wait_for_message('gazebo/model_states', ModelStates)
        #     for i in range(len(model.name)):
        #         if model.name[i] == 'obstacle':
        #             obstacle.model_name = 'obstacle'
        #             obstacle.pose = model.pose[i]
        #             obstacle.twist = Twist()
        #             obstacle.twist.angular.z = 0.5
        #             self.pub_model.publish(obstacle)
        #             time.sleep(0.1)

def main():
    # rospy.init_node('moving_obstacle')
    rclpy.init()
    moving = Moving()
    moving.moving()
    rclpy.shutdown()
    #rclpy.spin(moving)

if __name__ == '__main__':
    main()