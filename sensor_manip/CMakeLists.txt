cmake_minimum_required(VERSION 3.8)
project(sensor_manip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_update src/scan_update.cpp)
ament_target_dependencies(scan_update rclcpp sensor_msgs)


install(TARGETS
  scan_update
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
