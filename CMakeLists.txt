cmake_minimum_required(VERSION 3.0.2)
project(d-bot-m2m-task-executor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

##Generate services in the 'srv' folder
add_service_files(
   FILES
   AddTask.srv
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
