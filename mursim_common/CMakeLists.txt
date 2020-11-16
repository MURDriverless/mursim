cmake_minimum_required(VERSION 3.0.2)
project(mursim_common)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
)

add_message_files(DIRECTORY msg
    FILES
    car_state_msg.msg)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
