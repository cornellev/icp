# Copyright (C) 2024-2025 Cornell Electric Vehicles.
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.15)
project(CevIcp LANGUAGES C CXX)

set(PROJECT_VERSION 1.3.0)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(USE_SANITIZERS "Compile with ASan and UBSan" OFF)
option(CEV_ICP_BUILD_TOOLS "Build the legacy viewer, PLY demo, and benchmarks" ON)
option(CEV_ICP_BUILD_TESTS "Build the Eigen-only legacy unit suites" ON)
option(CEV_ICP_BUILD_ZENOH "Build the C++ Zenoh scan-matching node" OFF)

if(DEFINED ENV{CI})
  set(USE_SANITIZERS ON)
  set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

  # for clang thinking unused functions in headers are problematic
  # and also complaining about float-store
  if (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    add_compile_options(-Wno-error=unused-function -Wno-error=ignored-optimization-argument)
  endif()
endif()

# Add compile options
add_compile_options(-Wall -Wextra -g)
if(USE_SANITIZERS) 
  message(STATUS "Using address and UB sanitizers")
  add_compile_options(-fsanitize=address,undefined)
  add_link_options(-fsanitize=address,undefined)
endif()

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Find dependencies
find_package(Eigen3 REQUIRED)
if(CEV_ICP_BUILD_TOOLS)
find_package(SDL2)
find_package(PCL)

find_library(LIBCONFIG config)
find_library(LIBCMDAPP cmdapp)
find_library(LIBSDLWRAPPER sdlwrapper)

if(${SDL2_FOUND})
  message(STATUS "SDL2 found.")
else()
  message(WARNING "SDL2 not found.")
endif()

if(${PCL_FOUND})
  message(STATUS "PCL found. PCL-dependent targets can be built.")
  add_definitions(${PCL_DEFINITIONS})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
else()
  message(WARNING "PCL not found. PCL-dependent targets can NOT be built.")
endif()

if(LIBCONFIG)
  message(STATUS "libconfig found.")
else()
  message(WARNING "libconfig not found.")
  set(LIBCONFIG config)
endif()

if(LIBCMDAPP)
  message(STATUS "libcmdapp found.")
else()
  message(WARNING "libcmdapp not found.")
  set(LIBCMDAPP cmdapp)
endif()

if(LIBSDLWRAPPER)
  message(STATUS "libsdlwrapper found.")
else()
  message(WARNING "libsdlwrapper not found.")
  set(LIBSDLWRAPPER sdlwrapper)
endif()

endif()

# Library target
set(LIB_SOURCES
  lib/algo/kdtree.cpp
  lib/algo/quickselect.cpp
  lib/icp/icp.cpp
  lib/icp/impl/vanilla.cpp
  lib/icp/impl/trimmed.cpp
  lib/icp/impl/feature_aware.cpp
  lib/icp/impl/vanilla_3d.cpp
  lib/icp/impl/trimmed_3d.cpp
)
add_library(${PROJECT_NAME_LOWER} ${LIB_SOURCES})
add_library(CEV::${PROJECT_NAME} ALIAS ${PROJECT_NAME_LOWER})
target_include_directories(${PROJECT_NAME_LOWER} PUBLIC  
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME_LOWER} Eigen3::Eigen)

if(CEV_ICP_BUILD_TESTS AND "${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
  enable_testing()
  add_executable(test_suite tests/test.cpp)
  add_executable(test_suite_3d tests/test3d.cpp)
  target_link_libraries(test_suite PRIVATE ${PROJECT_NAME_LOWER} Eigen3::Eigen)
  target_link_libraries(test_suite_3d PRIVATE ${PROJECT_NAME_LOWER} Eigen3::Eigen)
  add_test(NAME legacy_2d COMMAND test_suite)
  add_test(NAME legacy_3d COMMAND test_suite_3d)
endif()

# Transport and JSON dependencies are private to the optional application.
if(CEV_ICP_BUILD_ZENOH)
  find_package(zenohc 1.10.1 EXACT REQUIRED)
  find_package(zenohcxx 1.10.1 EXACT REQUIRED)
  find_package(nlohmann_json 3.11 REQUIRED)
  find_package(Threads REQUIRED)
  add_library(icp_zenoh_support STATIC tools/icp_zenoh_support.cpp)
  target_include_directories(icp_zenoh_support PUBLIC tools)
  target_link_libraries(icp_zenoh_support PUBLIC ${PROJECT_NAME_LOWER}
    zenohcxx::zenohc nlohmann_json::nlohmann_json Threads::Threads)
  add_executable(icp_zenoh tools/icp_zenoh.cpp)
  target_link_libraries(icp_zenoh PRIVATE icp_zenoh_support)
  if(CEV_ICP_BUILD_TESTS AND "${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    add_executable(test_zenoh tests/test_zenoh.cpp)
    target_link_libraries(test_zenoh PRIVATE icp_zenoh_support)
    add_test(NAME zenoh_unit COMMAND test_zenoh --unit)
    add_test(NAME zenoh_loopback COMMAND test_zenoh --loopback)
    set_tests_properties(zenoh_unit zenoh_loopback PROPERTIES TIMEOUT 30)
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
      add_test(NAME zenoh_cli COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_zenoh_cli.py $<TARGET_FILE:icp_zenoh>)
      set_tests_properties(zenoh_cli PROPERTIES TIMEOUT 30)
    endif()
  endif()
endif()

# Main executable
if(CEV_ICP_BUILD_TOOLS AND "${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
  set(MAIN_SOURCES
    vis/main.cpp
    vis/lidar_view.cpp
    vis/view_config.cpp
    common/parse_scan.cpp
  )
  add_executable(main ${MAIN_SOURCES})
  target_link_libraries(main 
    ${PROJECT_NAME_LOWER}
    Eigen3::Eigen
    SDL2::SDL2
    ${LIBCMDAPP}
    ${LIBSDLWRAPPER}
    ${LIBCONFIG}
  )
  target_include_directories(main 
    PRIVATE 
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/vis
      ${CMAKE_CURRENT_SOURCE_DIR}/common
      ${EIGEN3_INCLUDE_DIRS}
      /usr/local/include/cmdapp
      /usr/local/include/config
      /usr/local/include/sdlwrapper
  )
  
  if(PCL_FOUND)
  set(TEST_SOURCES_PLY tests/test_ply.cpp)
  add_executable(test_suite_ply ${TEST_SOURCES_PLY})

  target_link_libraries(test_suite_ply
    ${PROJECT_NAME_LOWER}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
  )
  target_include_directories(test_suite_ply
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/common
      ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_suite_ply PRIVATE TEST)
  endif()

  set(BENCH_SOURCES bench/bench.cpp common/parse_scan.cpp)
  add_executable(bench_suite ${BENCH_SOURCES})
  target_link_libraries(bench_suite
    ${PROJECT_NAME_LOWER}
    ${LIBCONFIG}
  )
  target_include_directories(bench_suite
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/common
  )
endif()

# Include
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME_LOWER})

# Generate the CevIcpConfig.cmake file
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Generate the CevIcpConfigVersion.cmake file
write_basic_package_version_file(
  ${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME_LOWER} NAMESPACE CEV FILE ${PROJECT_NAME}Targets.cmake)

export(PACKAGE ${PROJECT_NAME})

# Install the targets and export them
install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE CEV::
  DESTINATION ${INSTALL_CONFIGDIR}
)

install(TARGETS ${PROJECT_NAME_LOWER}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME_LOWER} 
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install configuration files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

# Per https://gitlab.kitware.com/cmake/community/-/wikis/FAQ#can-i-do-make-uninstall-with-cmake
# uninstall target
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
