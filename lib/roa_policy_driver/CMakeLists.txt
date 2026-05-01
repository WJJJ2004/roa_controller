cmake_minimum_required(VERSION 3.16)
project(roa_policy_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ROA_BUILD_TESTS "Build tests" ON)

# ---------------- ONNX Runtime (manual link) ----------------
set(ORT_ROOT "/opt/onnxruntime/current" CACHE PATH "ONNX Runtime root dir")
set(ORT_INCLUDE_DIR "${ORT_ROOT}/include" CACHE PATH "ONNX Runtime include dir")
set(ORT_LIBRARY "${ORT_ROOT}/lib/libonnxruntime.so" CACHE FILEPATH "ONNX Runtime shared library")

if(NOT EXISTS "${ORT_INCLUDE_DIR}/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "onnxruntime_cxx_api.h not found: ${ORT_INCLUDE_DIR}/onnxruntime_cxx_api.h")
endif()
if(NOT EXISTS "${ORT_LIBRARY}")
  message(FATAL_ERROR "libonnxruntime.so not found: ${ORT_LIBRARY}")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ORT_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ORT_INCLUDE_DIR}"
)
# ------------------------------------------------------------


add_library(roa_policy_driver SHARED
  src/policy_driver.cpp
)

target_include_directories(roa_policy_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roa_policy_driver
  PUBLIC
    onnxruntime::onnxruntime
)
include(GNUInstallDirs)
target_compile_definitions(roa_policy_driver
  PUBLIC
    ROA_POLICY_DRIVER_SHARE_DIR=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\"
)

# ---- Install ----

install(TARGETS roa_policy_driver
  EXPORT roa_policy_driverTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY onnx/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/onnx
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# CMake package export
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/roa_policy_driverConfigVersion.cmake"
  VERSION 0.1.0
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/roa_policy_driverConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/roa_policy_driverConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/roa_policy_driver
)

install(EXPORT roa_policy_driverTargets
  FILE roa_policy_driverTargets.cmake
  NAMESPACE roa::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/roa_policy_driver
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/roa_policy_driverConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/roa_policy_driverConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/roa_policy_driver
)

if(ROA_BUILD_TESTS)
  enable_testing()

  set(ROA_DEFAULT_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/onnx/12dof/policy.onnx")

  add_executable(test_dimensions tests/test_dimensions.cpp)
  target_link_libraries(test_dimensions PRIVATE roa_policy_driver)
  add_test(NAME test_dimensions COMMAND test_dimensions "${ROA_DEFAULT_MODEL}")

  add_executable(bench_inference tests/bench_inference.cpp)
  target_link_libraries(bench_inference PRIVATE roa_policy_driver)


  add_executable(test_golden_io tests/test_golden_io.cpp)
  target_link_libraries(test_golden_io PRIVATE roa_policy_driver)
  add_test(NAME test_golden_io COMMAND test_golden_io
    "${ROA_DEFAULT_MODEL}"
    "${CMAKE_CURRENT_SOURCE_DIR}/onnx/12dof/data/golden_obs.bin"
    "${CMAKE_CURRENT_SOURCE_DIR}/onnx/12dof/data/golden_action.bin"
  )

  add_executable(test_stress tests/test_stress.cpp)
  target_link_libraries(test_stress PRIVATE roa_policy_driver)
  add_test(NAME test_stress COMMAND test_stress "${ROA_DEFAULT_MODEL}" "200000")

  add_executable(test_iface_roundtrip_12dof tests/test_iface_roundtrip_12dof.cpp)
  # 헤더-only라 링크 없어도 되지만, 통일/향후 확장 위해 링크 걸어도 OK
  target_link_libraries(test_iface_roundtrip_12dof PRIVATE roa_policy_driver)
  add_test(NAME test_iface_roundtrip_12dof COMMAND test_iface_roundtrip_12dof
    "${CMAKE_CURRENT_SOURCE_DIR}/onnx/12dof/data/golden_obs.bin"
  )
endif()
