cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconf)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure 
    rospy
)

generate_dynamic_reconfigure_options(
    cfg/Dynamic.cfg
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
