cmake_minimum_required(VERSION 3.16)
project(diffdrive_canbus LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Main hardware interface / CAN communication library
add_library(
  diffdrive_canbus
  SHARED
  hardware/CAN_comms.cpp
  hardware/spark_max.cpp
  hardware/Velocity_Controller.cpp
)

target_compile_features(diffdrive_canbus PUBLIC cxx_std_17)

target_include_directories(diffdrive_canbus PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  diffdrive_canbus PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(diffdrive_canbus PUBLIC
  serial
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(diffdrive_canbus PRIVATE
  "CANBUS_BUILDING_DLL"
)

# Standalone SPARK MAX test executable
add_executable(
  spark_max_test
  hardware/spark_max_test.cpp
)

target_compile_features(spark_max_test PUBLIC cxx_std_17)

target_include_directories(spark_max_test PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(spark_max_test
  diffdrive_canbus
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
  hardware_interface
  diffdrive_canbus.xml
)

# Install headers
install(
  DIRECTORY hardware/include/
  DESTINATION include
)

# Install robot description files
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch/config files
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

# Install library
install(
  TARGETS diffdrive_canbus
  EXPORT export_diffdrive_canbus
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install test executable
install(
  TARGETS spark_max_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# Exports
ament_export_targets(export_diffdrive_canbus HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()