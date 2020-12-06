cmake_minimum_required(VERSION 3.0.2)
project(mursim_gazebo_plugins)
set (CMAKE_CXX_STANDARD 11)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
            gazebo_msgs
            roscpp
            mur_common
            mursim_common)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    INCLUDE_DIRS
)

add_subdirectory(gz_vehicle_model_plugin)
