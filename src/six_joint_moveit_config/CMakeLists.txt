cmake_minimum_required(VERSION 3.8)
project(six_joint_moveit_config)

find_package(ament_cmake REQUIRED)

# Install MoveIt configuration and launch files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python robot motion script
install(
  PROGRAMS scripts/move_robot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_robot
)

ament_package()
