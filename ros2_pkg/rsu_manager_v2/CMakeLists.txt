cmake_minimum_required(VERSION 3.8)
project(rsu_manager_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roa_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(rsu_manager_v2_core
  src/rsu_lut.cpp
  src/rsu_model.cpp
)
target_include_directories(rsu_manager_v2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(rsu_manager_v2_node src/rsu_manager_v2_node.cpp)
target_link_libraries(rsu_manager_v2_node rsu_manager_v2_core Threads::Threads)
ament_target_dependencies(rsu_manager_v2_node rclcpp roa_interfaces realtime_tools ament_index_cpp)

install(TARGETS rsu_manager_v2_core rsu_manager_v2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch data DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rsu_core test/test_rsu_core.cpp)
  target_link_libraries(test_rsu_core rsu_manager_v2_core)
  target_compile_definitions(test_rsu_core PRIVATE
    RSU_V2_TEST_LUT="${CMAKE_CURRENT_SOURCE_DIR}/data/rsu_lut_v2.bin")
endif()

ament_package()
