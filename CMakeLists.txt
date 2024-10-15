# This package has no tests
cmake_minimum_required(VERSION 3.5)
project(tmc_dynpick_driver)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)

add_executable(dynpick_driver_node src/main.cpp)
target_include_directories(dynpick_driver_node PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(dynpick_driver_node geometry_msgs rclcpp)

install(
  TARGETS dynpick_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(geometry_msgs)
ament_export_include_directories()
ament_export_libraries()
ament_package()
