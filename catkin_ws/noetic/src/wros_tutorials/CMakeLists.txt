cmake_minimum_required(VERSION 2.8.3)
project(wros_tutorials)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/grasp_planning_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
