cmake_minimum_required(VERSION 3.0.2)
project(d_bot_m2m_task_executor)

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

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(PROGRAMS
    src/task_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
