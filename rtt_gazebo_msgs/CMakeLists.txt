cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs) 

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DeployRTTWithModel.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

