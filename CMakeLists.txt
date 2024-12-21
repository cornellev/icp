cmake_minimum_required(VERSION 3.10)
project(cevicp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CI "CI build" OFF)
if(CI)
  set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
endif()

# Add compile options
add_compile_options(-Wall -Wextra -pedantic)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(SDL2 REQUIRED)

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
  PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

# Main executable
set(MAIN_SOURCES
  src/main.cpp
  src/sim/lidar_view.cpp
  src/sim/view_config.cpp
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

# Debug/Release configuration
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_definitions(cevicp PRIVATE RELEASE_BUILD)
  target_compile_options(cevicp PRIVATE -O3)
else()
  target_compile_options(cevicp PRIVATE -g)
endif()

# Installation
install(TARGETS cevicp
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cev_icp
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
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
