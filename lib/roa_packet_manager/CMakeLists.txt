cmake_minimum_required(VERSION 3.8)
project(roa_packet_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roa_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/packet_manager.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  roa_interfaces
)

ament_package()