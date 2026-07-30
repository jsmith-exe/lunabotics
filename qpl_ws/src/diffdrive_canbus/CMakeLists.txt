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

# Find system LibSerial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# -------------------------------------------------------------------
# Hardware interface plugin library
# -------------------------------------------------------------------

add_library(
  diffdrive_canbus
  SHARED
  hardware/diffdrive_canbus_system.cpp
  hardware/can_system.cpp
  hardware/can_comms/can_comms.cpp
  hardware/can_comms/can_utils.cpp
  hardware/can_comms/serial_utils.cpp
  hardware/can_comms/waveshare_codec.cpp
  hardware/diffdrive_interface/helpers.cpp
  hardware/diffdrive_interface/can_comms_helpers.cpp
  hardware/diffdrive_interface/can_system.cpp
  hardware/diffdrive_interface/actuator.cpp
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
# Install headers and plugin XML
# -------------------------------------------------------------------

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  FILES diffdrive_canbus.xml
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