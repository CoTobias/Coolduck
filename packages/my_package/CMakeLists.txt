cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

