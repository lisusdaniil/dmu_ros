cmake_minimum_required(VERSION 2.8.3)
project(dmu_ros)

add_compile_options(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        roscpp
        tf
        message_generation
        )

add_message_files(FILES
        DMURaw.msg
        DMUCounter.msg
        )

generate_messages(DEPENDENCIES
        geometry_msgs
        sensor_msgs
        )

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS})


add_executable(dmu_node src/dmu_node.cpp
        src/DMU.cpp
        include/DMU.h)
target_link_libraries(dmu_node ${catkin_LIBRARIES})
add_dependencies(dmu_node
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_generate_messages_cpp
        )

add_executable(simulation_node src/simulation_node.cpp)
target_link_libraries(simulation_node ${catkin_LIBRARIES})

add_executable(delta_computations src/delta_computations.cpp)
target_link_libraries(delta_computations ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS dmu_node simulation_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

