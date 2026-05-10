cmake_minimum_required(VERSION 3.16)
project(diffdrive_canbus LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  diffdrive_canbus
  SHARED
  hardware/CAN_comms.cpp
  hardware/spark_max.cpp
  hardware/diffdrive_canbus_system.cpp
)

target_compile_features(diffdrive_canbus PUBLIC cxx_std_17)

target_include_directories(diffdrive_canbus PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  diffdrive_canbus PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  serial
)

target_link_libraries(diffdrive_canbus PUBLIC
  serial
)

target_compile_definitions(diffdrive_canbus PRIVATE
  DIFFDRIVE_CANBUS_BUILDING_DLL
)

add_executable(
  spark_max_test
  hardware/spark_max_test.cpp
)

target_compile_features(spark_max_test PUBLIC cxx_std_17)

target_include_directories(spark_max_test PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  spark_max_test PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  serial
)

target_link_libraries(spark_max_test PUBLIC
  diffdrive_canbus
  serial
)

pluginlib_export_plugin_description_file(
  hardware_interface
  diffdrive_canbus.xml
)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    diffdrive_canbus
    spark_max_test
  EXPORT export_diffdrive_canbus
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_diffdrive_canbus HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  serial
)

ament_package()