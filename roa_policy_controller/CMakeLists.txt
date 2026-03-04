cmake_minimum_required(VERSION 3.8)
project(roa_policy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roa_interfaces REQUIRED)

find_package(roa_policy_driver REQUIRED)
target_link_libraries(roa_policy_controller
  PRIVATE
    roa::roa_policy_driver
)

add_library(roa_policy_controller SHARED
  src/policy_controller.cpp
  src/plugin_export.cpp
)

target_include_directories(roa_policy_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roa_policy_controller
  rclcpp rclcpp_lifecycle
  controller_interface pluginlib realtime_tools
  std_msgs sensor_msgs geometry_msgs
  roa_interfaces
)

# 링크: roa_policy_driver (소스 빌드 옵션일 때)
if(ROA_LINK_POLICY_DRIVER_FROM_SOURCE)
  target_link_libraries(roa_policy_controller
    roa_policy_driver  # roa_policy_driver 쪽에서 export하는 타겟 이름에 맞춰야 함
  )
endif()

# pluginlib export
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(TARGETS roa_policy_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()