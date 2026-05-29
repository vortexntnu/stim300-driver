find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ament_add_gtest(check_datasheet_constanst
${CMAKE_CURRENT_SOURCE_DIR}/check_stim300_constants.cpp
)
target_include_directories(check_datasheet_constanst
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(check_datasheet_constanst
  stim300_driver_lib
)
ament_target_dependencies(check_datasheet_constanst
  rclcpp
  sensor_msgs
)

ament_add_gmock(check_driver_stim300
  ${CMAKE_CURRENT_SOURCE_DIR}/check_driver_stim300.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mock_serial_driver.h
)
target_include_directories(check_driver_stim300
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(check_driver_stim300
  stim300_driver_lib
)
ament_target_dependencies(check_driver_stim300
  rclcpp
  sensor_msgs
)