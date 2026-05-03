cmake_minimum_required(VERSION 3.8)
project(roa_common)

find_package(ament_cmake REQUIRED)
find_package(roa_packet_manager REQUIRED)

ament_export_dependencies(roa_packet_manager)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
