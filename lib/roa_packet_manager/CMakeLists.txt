cmake_minimum_required(VERSION 3.8)
project(roa_packet_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roa_interfaces REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  roa_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp roa_interfaces)
ament_export_include_directories(include)

ament_package()
