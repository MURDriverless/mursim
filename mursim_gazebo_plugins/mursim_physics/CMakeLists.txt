cmake_minimum_required(VERSION 3.0.2)
project(mursim_physics)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mur_common
    mursim_common)

find_package(gazebo REQUIRED)

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mursim_physics SHARED
            src/mursim_physics.cpp
            src/vehicle.cpp
            src/wheel.cpp
            src/axle.cpp)

target_include_directories(mursim_physics PUBLIC include 
                           ${CMAKE_SOURCE_DIR} 
                           ${catkin_LIBRARY_DIRS})

target_link_libraries(mursim_physics 
                      ${GAZEBO_LIBRARIES} 
                      ${IGNITION-MSGS_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      yaml-cpp)

install(TARGETS mursim_physics
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
