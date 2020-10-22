cmake_minimum_required(VERSION 3.0.2)
project(mursim_physics)

find_package(catkin REQUIRED COMPONENTS
    roscpp)

find_package(gazebo)

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(mursim_physics SHARED
    src/mursim_physics.cpp)
