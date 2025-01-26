cmake_minimum_required(VERSION 3.14)
project(cev_icp CXX)

set(PROJECT_VERSION 1.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile options
add_compile_options(-Wall -Wextra -pedantic)

option(CI "CI build" OFF)
if(CI)
  set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
  # simple_test
  add_compile_options(-Wno-error=unused-function)
  if(CMAKE_COMPILER_IS_GNUCXX)
    # eigen
    add_compile_options(-Wno-error=class-memaccess) 
  endif()
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Library target
set(LIB_SOURCES
  lib/algo/kdtree.cpp
  lib/algo/quickselect.cpp
  lib/icp/icp.cpp
  lib/icp/driver.cpp
  lib/icp/geo.cpp
  lib/icp/impl/vanilla.cpp
  lib/icp/impl/trimmed.cpp
  lib/icp/impl/feature_aware.cpp
)
add_library(cevicp STATIC ${LIB_SOURCES})
target_include_directories(cevicp 
  PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
)

# Main executable
set(MAIN_SOURCES
  vis/main.cpp
  vis/lidar_view.cpp
  vis/view_config.cpp
  common/parse_scan.cpp
)
add_executable(main ${MAIN_SOURCES})
target_link_libraries(main 
  cevicp
  SDL2::SDL2
  libcmdapp.a
  libsdlwrapper.a
  libconfig.a
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

set(TEST_SOURCES tests/test.cpp)
add_executable(test_suite ${TEST_SOURCES})
target_link_libraries(test_suite
  cevicp
)
target_include_directories(test_suite
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include/simple_test
)
target_compile_definitions(test_suite PRIVATE TEST)

set(BENCH_SOURCES bench/bench.cpp common/parse_scan.cpp)
add_executable(bench_suite ${BENCH_SOURCES})
target_link_libraries(bench_suite
  cevicp
  libconfig.a
)
target_include_directories(bench_suite
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# Debug/Release configuration
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_definitions(cevicp PRIVATE RELEASE_BUILD)
  target_compile_options(cevicp PRIVATE -O3)
else()
  target_compile_options(cevicp PRIVATE -g)
endif()

# Installation
include(CMakePackageConfigHelpers)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/cevicp)

# Generate the CevIcpConfigVersion.cmake file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/CevIcpConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Generate the CevIcpConfig.cmake file
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/CevIcpConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/CevIcpConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Install the targets and export them
install(TARGETS cevicp
  EXPORT CevIcpTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(EXPORT CevIcpTargets
  FILE CevIcpTargets.cmake
  NAMESPACE CEV::
  DESTINATION ${INSTALL_CONFIGDIR}
)

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cev_icp
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install configuration files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/CevIcpConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/CevIcpConfigVersion.cmake
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
