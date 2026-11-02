cmake_minimum_required(VERSION 3.8)
project(roa_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_joint_limits test/test_joint_limits.cpp)
  target_link_libraries(test_joint_limits PRIVATE ${PROJECT_NAME})
  add_test(NAME test_joint_limits COMMAND test_joint_limits)
  add_executable(test_init_pose test/test_init_pose.cpp)
  target_link_libraries(test_init_pose PRIVATE ${PROJECT_NAME})
  ament_target_dependencies(test_init_pose PUBLIC roa_packet_manager)
  add_test(NAME test_init_pose COMMAND test_init_pose)
endif()

ament_export_include_directories(include)
ament_package()
