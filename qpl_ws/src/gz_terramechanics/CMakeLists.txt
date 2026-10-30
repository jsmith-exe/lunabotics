cmake_minimum_required(VERSION 3.10)
project(gz_terramechanics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(TerramechanicsSystem MODULE src/TerramechanicsSystem.cc)

target_link_libraries(TerramechanicsSystem PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  yaml-cpp
  OpenMP::OpenMP_CXX
)

install(TARGETS TerramechanicsSystem LIBRARY DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Puts lib/ on GZ_SIM_SYSTEM_PLUGIN_PATH when the workspace is sourced
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
