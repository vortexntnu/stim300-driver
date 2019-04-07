cmake_minimum_required(VERSION 3.5.1)
project(driver_stim300)

set(CMAKE_CXX_STANDARD 14)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)


add_library(stim300_driver_lib
        src/driver_stim300.cpp
        src/datagram_parser.cpp
        src/serial_unix.cpp)

target_include_directories(stim300_driver_lib
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )
target_link_libraries(stim300_driver_lib)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )


add_executable(stim300_driver_node src/stim300_driver_node.cpp)
target_include_directories(stim300_driver_node
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${catkin_INCLUDE_DIRS})
target_link_libraries(stim300_driver_node PRIVATE stim300_driver_lib ${catkin_LIBRARIES})

