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
            <robot name="obstacle">
                <!-- Base Link -->
                <link name="base_link">
                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <box size="0.5 0.5 0.5"/>
                        </geometry>
                    </collision>

                    <visual>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <box size="0.5 0.5 0.5"/>
                        </geometry>
                        <material name="blue">
                            <color rgba="0 0 1 1"/>
                        </material>
                    </visual>

                    <inertial>
                        <origin xyz="0 0 0.25" rpy="0 0 0"/>
                        <mass value="1"/>
                        <inertia
                            ixx="0.166667" ixy="0.0" ixz="0.0"
                            iyy="0.1666670" iyz="0.0"
                            izz="0.1666670"/>
                    </inertial>
                </link>
            </robot>
            
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
            # update the x position based on the direction
            # x_pos += direction * vel.linear.x * 0.1  # integrate position over time
            # vel.linear.y = 0.0
            # vel.linear.z = 0.0
            # vel.angular.x = 0.0
            # vel.angular.y = 0.0
            # vel.angular.z = 0.0
            # # if the box reaches the end of the path, reverse direction
            # if x_pos > 2.0:
            #     direction = -1
            # elif x_pos < 0.0:
            #     direction = 1
            # # set the x velocity based on the current direction
            # vel.linear.x = direction * abs(vel.linear.x)
            # self.vel_pub.publish(vel)
            # self.node.get_logger().info(f'Moving obstacle to x={x_pos:.2f}')
            # rclpy.spin_once(self.node, timeout_sec=0.1)
    



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



# #!/usr/bin/env python3

# import rclpy
# from gazebo_msgs.srv import SpawnEntity
# from geometry_msgs.msg import Twist

# class Moving:
#     def __init__(self):
#         super().__init__('move_obstacle')

#         self.node = rclpy.create_node('moving_obstacle')
#         self.client = self.node.create_client(SpawnEntity, '/spawn_entity')
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.node.get_logger().info('Service not available, waiting again...')

#         self.vel_pub = self.node.create_publisher(Twist, '/obstacle/cmd_vel', 10)
#         self.moving()

#     def moving(self):
#         # Spawn a new model in Gazebo
#         model_xml = """
#             <robot name="obstacle">
#                 <!-- Base Link -->
#                 <link name="base_link">
#                     <collision>
#                     <origin xyz="0 0 0" rpy="0 0 0"/>
#                     <geometry>
#                         <box size="0.5 0.5 0.5"/>
#                     </geometry>
#                     </collision>

#                     <visual>
#                     <origin xyz="0 0 0" rpy="0 0 0"/>
#                     <geometry>
#                         <box size="0.5 0.5 0.5"/>
#                     </geometry>
#                     <material name="blue">
#                         <color rgba="0 0 1 1"/>
#                     </material>
#                     </visual>

#                     <inertial>
#                     <origin xyz="0 0 0.25" rpy="0 0 0"/>
#                     <mass value="1"/>
#                     <inertia
#                         ixx="1.0" ixy="0.0" ixz="0.0"
#                         iyy="1.0" iyz="0.0"
#                         izz="1.0"/>
#                     </inertial>
#                 </link>
#             </robot>
            
#             """
                    
#     #     <?xml version="1.0"?>
#     #     <robot name="my_box_model">
#     #     <link name="base_link">
#     #         <inertial>
#     #         <mass value="1.0"/>
#     #         <origin rpy="0 0 0" xyz="0 0 0"/>
#     #         <!--<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>-->
#     #         </inertial>
#     #         <visual>
#     #         <geometry>
#     #             <box size="0.5 0.5 0.5"/>
#     #         </geometry>
#     #         <material name="blue">
#     #             <color rgba="0 0 1 1"/>
#     #         </material>
#     #         </visual>
#     #     </link>
#     #     <gazebo>
#     #         <plugin name="gazebo_ros_controller_plugin" filename="libgazebo_ros_controller_plugin.so">
#     #         <robotNamespace>/my_box_model</robotNamespace>
#     #         <legacyModeNS>false</legacyModeNS>
#     #         <legacyControllerType>transmission_interface/SimpleTransmission</legacyControllerType>
#     #         <legacyPluginType>gazebo_ros_control/DefaultRobotHWSim</legacyPluginType>
#     #         <robotSimType>gazebo_ros_control/DefaultRobotSim</robotSimType>
#     #         <hardwareInterface>hardware_interface::EffortJointInterface</hardwareInterface>
#     #         <pluginLibName>libmy_box_model_ros_control_plugin.so</pluginLibName>
#     #         <jointLimitInterface>true</jointLimitInterface>
#     #         </plugin>
#     #     </gazebo>
#     #     </robot>
#     # """
#     #     <model name='unit_box'>
#     #     <pose>3.94821 13.7512 0.546432 0.001273 -0 0</pose>
#     #     <scale>1 5.6083 1</scale>
#     #     <link name='link'>
#     #       <pose>3.94821 13.7512 0.546432 0.001273 -0 0</pose>
#     #       <velocity>0 0 0 0 -0 0</velocity>
#     #       <acceleration>0 0 0 0 -0 0</acceleration>
#     #       <wrench>0 0 0 0 -0 0</wrench>
#     #     </link>
#     #   </model>
        
#         #"""
#         #<robot name="obstacle">
#         #  <link name="base_link"/>
#         #</robot>
#         #"""
#         spawn_req = SpawnEntity.Request()
#         spawn_req.name = 'obstacle'
#         spawn_req.xml = model_xml
#         self.client.call_async(spawn_req)

#         # Move the model
#         while rclpy.ok():
#             vel = Twist()
#             #vel.angular.z = 0.005
           
#             # Set the model's linear and angular velocities to zero
#             vel.linear.x = 0.0
#             vel.linear.y = 0.0
#             vel.linear.z = 0.0
#             vel.angular.x = 0.0
#             vel.angular.y = 0.0
#             vel.angular.z = 5.0
#             self.vel_pub.publish(vel)
#             self.node.get_logger().info('Moving obstacle')
#             rclpy.spin_once(self.node, timeout_sec=1)

# def main():
#     rclpy.init()
#     moving = Moving()
#     rclpy.spin(moving.node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()