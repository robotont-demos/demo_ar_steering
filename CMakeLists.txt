cmake_minimum_required(VERSION 3.0.2)
project(ar_steering)

find_package(catkin REQUIRED COMPONENTS
  robotont_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config launch src DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})