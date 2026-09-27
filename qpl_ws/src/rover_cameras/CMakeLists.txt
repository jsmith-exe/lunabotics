cmake_minimum_required(VERSION 3.8)
project(rover_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET libavcodec libavutil)

# ROS-free core: libavcodec wrapper and OpenCV conversion (unit-tested on their own)
add_library(stream_core STATIC src/h264_encoder.cpp src/convert.cpp)
set_target_properties(stream_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(stream_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(stream_core PUBLIC PkgConfig::LIBAV ${OpenCV_LIBS})
target_compile_features(stream_core PUBLIC cxx_std_17)

# The compression node: rover_cameras::StreamEncoder (component + stream_encoder_node executable)
add_library(stream_encoder SHARED src/stream_encoder_node.cpp)
target_link_libraries(stream_encoder stream_core)
ament_target_dependencies(stream_encoder
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs ffmpeg_image_transport_msgs)
rclcpp_components_register_node(stream_encoder
  PLUGIN "rover_cameras::StreamEncoder"
  EXECUTABLE stream_encoder_node)

install(TARGETS stream_encoder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_python_install_package(${PROJECT_NAME})
# Python nodes, installed without the .py so `ros2 run rover_cameras camera_sim` works
install(PROGRAMS scripts/camera_sim.py DESTINATION lib/${PROJECT_NAME} RENAME camera_sim)
install(PROGRAMS scripts/imu_optical_to_standard.py
  DESTINATION lib/${PROJECT_NAME} RENAME imu_optical_to_standard)
install(DIRECTORY launch config calibration
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stream_core test/test_stream_core.cpp)
  target_link_libraries(test_stream_core stream_core)
endif()

ament_package()
