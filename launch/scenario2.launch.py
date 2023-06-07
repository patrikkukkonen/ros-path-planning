# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from launch.actions import ExecuteProcess



def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # This package
    pkg_dir = get_package_share_directory('path_finding')

    # turtlebot3 gazebo worlds and models
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='2.50'),
            'y': LaunchConfiguration('y_pose', default='-2.00'),
            'z': LaunchConfiguration('z_pose', default='0.05'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_dir, 'maps', 'maze_v2.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Modified: Use custom made nav2_params file where initial pose is set
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'nav2_params_initial.yaml'), # Fix noise in laser scan ---- FIXED, was bad world file
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            pkg_dir, 'rviz', 'nav2_config.rviz'), #bringup_dir, 'rviz', 'nav2_default_view.rviz'), # pkg_dir, 'rviz', 'nav2_config.rviz'),
        description='Full path to the RVIZ config file to use') # TODO: improve the replanning of the path

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False', # FALSE == LAUNCH GAZEBO
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        # worlds/turtlebot3_worlds/waffle.model')
        default_value=os.path.join(pkg_dir, 'worlds', 'maze3.world'), # TODO: FIX THIS
        description='Full path to world model file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle_piiiiiiiiiiii', # Use Waffle Pi instead of waffle
        description='name of the robot')

    # Robot sdf file
    # declare_robot_sdf_cmd = DeclareLaunchArgument(
    #     'robot_sdf',
    #     default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'), # default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
    #     description='Full path to robot sdf file to spawn the robot in gazebo')
    
    # Get the SDF file
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models',
            model_folder,
            'model.sdf'
        )
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    # Robot URDF file
    # urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')  # urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    # with open(urdf, 'r') as infp:
    #     robot_description = infp.read()

    # Get the urdf file
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
         robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())
    
    # start the demo autonomy task
    waypoint_follower_cmd = Node(
        package='path_finding',
        executable='waypoint_follower.py',
        emulate_tty=True,
        output='screen')
    
    # Spawner node for spawning objects
    spawner_cmd = Node(
        package='path_finding',
        executable='scenario2_objects.py',
        output='screen')
    
    # Create a folder name based on the current timestamp
    folder_name = "dijkstra"
    # Specify the path and filename for saving the ROS bag
    bag_file_path = '/bag_files/scenario1/' + folder_name + '/bagfile.bag'
    # Launch node to start recording the ROS bag
    rosbag_recorder = ExecuteProcess(
        #cmd=['ros2', 'bag', 'record', '-a'],
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--all', '--include-hidden-topics'],
        output='screen'
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)


    # Waypoint follower node
    #ld.add_action(waypoint_follower_node)
    # Delay the start of the waypoint following node by 10 seconds so that gazebo has time to start
    ld.add_action(TimerAction(
        period=10.0,
        actions=[waypoint_follower_cmd]
    ))

    # Start rosbag record
    # ld.add_action(TimerAction(
    #      period=10.0,
    #         actions=[rosbag_recorder]
    # ))
    ld.add_action(rosbag_recorder)

    # Delay the start of the object spawner node by 25 seconds
    ld.add_action(TimerAction(
        period=45.0,
        actions=[spawner_cmd]
    ))

    # print(pkg_dir)
    # print(pkg_dir)
    # print(pkg_dir)
    # print(pkg_dir)
    # print(pkg_dir)
    # print(pkg_dir)
    # print(pkg_dir)

    # print(bringup_dir)

    # print(launch_dir)

   
    #quit()

    return ld
