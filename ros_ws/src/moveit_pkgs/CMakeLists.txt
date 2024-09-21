cmake_minimum_required(VERSION 3.1.3)
project(moveit_pkgs)

# Specify required packages here
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  # Add other dependencies as needed
)

catkin_package()

# Install launch files excluding the setup_assistant.launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "setup_assistant.launch" EXCLUDE
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
