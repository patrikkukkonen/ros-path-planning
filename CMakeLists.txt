cmake_minimum_required(VERSION 3.8)
project(path_finding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # default

find_package(rclcpp REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


# spawn and move object in gazebo
#add_executable(moving2 scritps/moving2.py) # Add executable
#ament_target_dependencies(moving2 rclcpp) # Add dependencies

# # Add the executable for your time-to-goal publisher
# add_executable(time_to_goal_publisher scripts/time_to_goal_publisher.cpp)

# # Link against the necessary libraries
# ament_target_dependencies(time_to_goal_publisher rclcpp std_msgs)

# # Install the executable
# install(TARGETS
#   time_to_goal_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )


# Install python scripts
install(PROGRAMS
  #scripts/moving2.py
  #scripts/moving3.py
  #scripts/moving4.py
  #scripts/moving5.py
  scripts/waypoint_follower.py
  # scripts/waypoint_follower2.py
  #scripts/robot_navigator.py
  scripts/object_spawner.py
  #scripts/object_spawner2.py
  #scripts/object_spawner3.py
  #scripts/object_spawner4.py
  #scripts/object_spawner5.py

  scripts/scenario1_objects.py
  scripts/scenario2_objects.py
  scripts/scenario3_objects.py
  scripts/scenario4_objects.py
  scripts/scenario5_objects.py

  # script to execute all launch files
  #scripts/time_to_goal_publisher.py
  scripts/collision_count.py

  scripts/distance.py
  scripts/distance2.py

  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY launch maps models rviz worlds params
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
