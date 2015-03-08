cmake_minimum_required(VERSION 2.8.3)
project(kinect_baxter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  tf2_ros
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

catkin_package(
  LIBRARIES kinect_baxter
  CATKIN_DEPENDS geometry_msgs rospy std_msgs tf2_ros
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
