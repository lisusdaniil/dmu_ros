cmake_minimum_required(VERSION 2.8.3)
project(dmu_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++0x)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        roscpp
        tf
        )

catkin_package()

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(DMU11_LIB
        src/DMU11.cpp
        include/DMU11.h
        )

add_executable(${PROJECT_NAME}_node src/dmu_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES} DMU11_LIB
        )


