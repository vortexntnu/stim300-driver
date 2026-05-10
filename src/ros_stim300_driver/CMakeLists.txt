set(ROS_DRIVER_LIB ros_stim300_driver_lib)

add_library(
    ${ROS_DRIVER_LIB}
    SHARED
)

target_sources(
    ${ROS_DRIVER_LIB}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_stim300_driver.cpp
)

ament_target_dependencies(
    ${ROS_DRIVER_LIB}
    PUBLIC
    rclcpp
    rclcpp_components
    sensor_msgs
)

target_link_libraries(
  ${ROS_DRIVER_LIB}
  PUBLIC
  stim300_driver_lib
)

target_include_directories(
    ${ROS_DRIVER_LIB}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
)

rclcpp_components_register_node(
  ${ROS_DRIVER_LIB}
  PLUGIN "Stim300DriverNode"
  EXECUTABLE ros_stim300_driver_node
)

install(TARGETS
    ${ROS_DRIVER_LIB}
    DESTINATION lib
)

install(TARGETS
    ros_stim300_driver_node
    DESTINATION lib/${PROJECT_NAME}
)
