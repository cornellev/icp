cmake_minimum_required(VERSION 3.14)
project(cev_icp CXX)

set(PROJECT_VERSION 1.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile options
add_compile_options(-Wall -Wextra -pedantic)

option(LIB_ONLY "Build library only" OFF)

# Find dependencies
find_package(Eigen3 REQUIRED)
if(NOT LIB_ONLY)
  find_package(SDL2 REQUIRED)

  # Specify PCL path and find PCL
  set(PCL_DIR "/opt/homebrew/share/pcl-1.14")
  find_package(PCL 1.14 REQUIRED COMPONENTS common io visualization)

  include_directories(${PCL_INCLUDE_DIRS} /usr/local/include)
  link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
endif()

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
  lib/icp/impl/vanilla_3d.cpp
  common/parse_ply.cpp  # Add the PLY parser source file
)
add_library(cevicp STATIC ${LIB_SOURCES})
target_include_directories(cevicp 
  PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cevicp ${PCL_LIBRARIES})

if(NOT LIB_ONLY)
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
  
  set(TEST_SOURCES_3D tests/test3d.cpp)
  add_executable(test_suite3d ${TEST_SOURCES_3D})
  target_link_libraries(test_suite3d
    cevicp
  )
  target_include_directories(test_suite3d
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      /usr/local/include/simple_test
      ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_suite3d PRIVATE TEST)

  # Add the new test executable for test_ply
  set(TEST_SOURCES_PLY tests/test_ply.cpp)
  add_executable(test_ply ${TEST_SOURCES_PLY})
  target_link_libraries(test_ply
    cevicp
  )
  target_include_directories(test_ply
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/common
      /usr/local/include/simple_test
      ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(test_ply PRIVATE TEST)

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
endif()

# Debug/Release configuration
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_definitions(cevicp PRIVATE RELEASE_BUILD)
  target_compile_options(cevicp PRIVATE -O3)
  
  if(NOT LIB_ONLY)
    target_compile_definitions(main PRIVATE RELEASE_BUILD)
    target_compile_options(main PRIVATE -O3)
    
    target_compile_definitions(test_suite PRIVATE RELEASE_BUILD)
    target_compile_options(bench_suite PRIVATE -O3)

    target_compile_definitions(bench_suite PRIVATE RELEASE_BUILD)
    target_compile_options(bench_suite PRIVATE -O3)
  endif()
else()
  target_compile_options(cevicp PRIVATE -g)

  if(NOT LIB_ONLY)
    target_compile_options(main PRIVATE -g)
    target_compile_options(test_suite PRIVATE -g)
    target_compile_options(bench_suite PRIVATE -g)
  endif()
endif()

# Installation
if(NOT LIB_ONLY)
  include(CMakePackageConfigHelpers)
  set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/cevicp)

  # Generate the CevIcpConfig.cmake file
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/CevIcpConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CevIcpConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
  )

  # Generate the CevIcpConfigVersion.cmake file
  write_basic_package_version_file(
    CevIcpConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
  )

  export(TARGETS cevicp NAMESPACE CevIcp FILE CevIcpTargets.cmake)

  export(PACKAGE CevIcp)

  # Install the targets and export them
  install(EXPORT CevIcpTargets
    NAMESPACE CevIcp::
    DESTINATION ${INSTALL_CONFIGDIR}
  )

  install(TARGETS cevicp
    EXPORT CevIcpTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
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
endif()
