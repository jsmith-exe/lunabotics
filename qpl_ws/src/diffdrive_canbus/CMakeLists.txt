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

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# -------------------------------------------------------------------
# Find system LibSerial
# -------------------------------------------------------------------

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# -------------------------------------------------------------------
# Hardware interface plugin library
# -------------------------------------------------------------------

add_library(
  diffdrive_canbus
  SHARED
  hardware/diffdrive_canbus_system.cpp
  hardware/CAN_comms.cpp
  hardware/spark_max.cpp
)

target_compile_features(diffdrive_canbus PUBLIC cxx_std_17)

target_include_directories(diffdrive_canbus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(
  diffdrive_canbus
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_directories(
  diffdrive_canbus
  PUBLIC
  ${LIBSERIAL_LIBRARY_DIRS}
)

target_link_libraries(
  diffdrive_canbus
  ${LIBSERIAL_LIBRARIES}
)

target_compile_options(
  diffdrive_canbus
  PUBLIC
  ${LIBSERIAL_CFLAGS_OTHER}
)

target_compile_definitions(diffdrive_canbus PRIVATE
  CANBUS_BUILDING_DLL
)

# Export plugin description to pluginlib
pluginlib_export_plugin_description_file(
  hardware_interface
  diffdrive_canbus.xml
)

# -------------------------------------------------------------------
# Test executable
# -------------------------------------------------------------------

add_executable(
  spark_max_test
  hardware/spark_max_test.cpp
)

target_compile_features(spark_max_test PUBLIC cxx_std_17)

target_include_directories(spark_max_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(
  spark_max_test
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_directories(
  spark_max_test
  PUBLIC
  ${LIBSERIAL_LIBRARY_DIRS}
)

target_link_libraries(
  spark_max_test
  diffdrive_canbus
  ${LIBSERIAL_LIBRARIES}
)

target_compile_options(
  spark_max_test
  PUBLIC
  ${LIBSERIAL_CFLAGS_OTHER}
)

# -------------------------------------------------------------------
# Install headers
# -------------------------------------------------------------------

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

# -------------------------------------------------------------------
# Install plugin XML
# -------------------------------------------------------------------

install(
  FILES diffdrive_canbus.xml
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Install descriptions / bringup files
# -------------------------------------------------------------------

install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Install library
# -------------------------------------------------------------------

install(
  TARGETS diffdrive_canbus
  EXPORT export_diffdrive_canbus
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -------------------------------------------------------------------
# Install executable
# -------------------------------------------------------------------

install(
  TARGETS spark_max_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Exports
# -------------------------------------------------------------------

ament_export_targets(export_diffdrive_canbus HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()