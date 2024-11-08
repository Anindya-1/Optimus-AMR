cmake_minimum_required(VERSION 3.8)
project(diffdrive_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_manip REQUIRED)
find_package(pf_description REQUIRED)

# Find libmodbus
find_library(MODBUS_LIBRARY modbus REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  diffdrive_bot
  SHARED
  hardware/src/diffbot_system.cpp
  hardware/src/motor_handler.cpp
)
target_compile_features(diffdrive_bot PUBLIC cxx_std_17)
target_include_directories(diffdrive_bot PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>"
  "$<INSTALL_INTERFACE:include/diffdrive_bot>"
)
target_link_libraries(diffdrive_bot PUBLIC ${rclcpp_LIBRARIES} ${MODBUS_LIBRARY})
ament_target_dependencies(
  diffdrive_bot PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_bot.xml)


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY
  bringup/launch bringup/config bringup/worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY description/urdf description/rviz description/ros2_control
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS diffdrive_bot
  EXPORT export_diffdrive_bot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_bot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
