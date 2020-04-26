cmake_minimum_required(VERSION 3.1)
project( p3d )

# message("Your C++ compiler supports these C++ features:")
# foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
#   message("${i}")
# endforeach()

include ( CMakePackageConfigHelpers )

#==============================================================================

option (BUILD_SHARED_LIBS "Build p3d as a shared library" ON)
option(USE_OPENMP "Use OpenMP" ON)

#==============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "compiling in debug")
    set(CMAKE_DEBUG_POSTFIX -debug)
    add_definitions(-DPOLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "compiling in release")
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty 
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty_bin 
    ${CMAKE_PREFIX_PATH}
)

set(CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty 
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty_bin 
    ${CMAKE_MODULE_PATH}
)

# ----- Sources ----------------------------------------------

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/core.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/core.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/logger.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/serialization.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/serialization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/commands.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/commands.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/command_manager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/command_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/image/feature_extraction.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/image/feature_extraction.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/multiview/autocalib.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/multiview/autocalib.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/multiview/bundle_adjustment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/multiview/bundle_adjustment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/multiview/bundle_params.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/dense_matching.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/dense_matching.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/fundmat.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/fundmat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/matching.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/matching.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/rectification.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/stereo/rectification.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/tasks.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/tasks.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/project_settings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/project_settings.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/affine_camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/affine_camera.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/image.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/image.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/image_pair.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/image_pair.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/point_cloud_container.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/point_cloud_container.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/point_cloud.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/data/point_cloud.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/project.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/p3d/project.h
)

set (p3d_lib p3d)
add_library (${p3d_lib} ${SOURCES})
target_compile_features(${p3d_lib} PRIVATE cxx_std_17)

target_include_directories(${p3d_lib} SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty)
target_include_directories(${p3d_lib} SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/)

# ----- Threading--------------------------------------------

if(USE_OPENMP)
    find_package(OpenMP QUIET)
    if (OPENMP_FOUND)
        message( STATUS "OpenMP - FOUND" )
        add_definitions(-DWITH_OPENMP)
    else()
        message( STATUS "OpenMP - not found" )
    endif()
endif()

# ----- Add Eigen --------------------------------------------

find_package(Eigen3 REQUIRED QUIET)
if (Eigen3_FOUND)
    message("Eigen3: found")
    target_include_directories(${p3d_lib} PUBLIC ${Eigen3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3: not found")
endif()

# ----- Add OpenCV --------------------------------------------

find_package(opencv REQUIRED QUIET)
if (opencv_FOUND)
    message("OpenCV: found")
    target_include_directories(${p3d_lib} PUBLIC ${opencv_INCLUDE_DIR})
else()
    message(FATAL_ERROR "OpenCV: not found")
endif()

# ----- Add NLopt --------------------------------------------

find_package(NLopt REQUIRED QUIET)
if (NLopt_FOUND)
    message("nlopt: found")
    target_include_directories(${p3d_lib} PRIVATE ${NLopt_INCLUDE_DIR})
else()
    message(FATAL_ERROR "nlopt: not found")
endif()

# ----- Add ceres --------------------------------------------

find_package(ceres-solver REQUIRED QUIET)
if (ceres-solver_FOUND)
    message("ceres-solver: found")
    target_include_directories(${p3d_lib} PRIVATE ${ceres-solver_INCLUDE_DIR})
    target_include_directories(${p3d_lib} PRIVATE ${ceres-solver_LIB_DIRS}/../include/ceres/internal/miniglog)
else()
    message(FATAL_ERROR "ceres-solver: not found")
endif()

message(dfdsfsdj ${ceres-solver_LIBS})
target_link_libraries(${p3d_lib}
    PUBLIC
        ${opencv_LIBS}
        OpenMP::OpenMP_CXX
        ${Eigen3_LIBS}
    PRIVATE
        ${NLopt_LIBS}
        ${NLopt_SYSTEM_LIBS}
        ${ceres-solver_LIBS}
    )

if (BUILD_SHARED_LIBS)
    target_compile_definitions (${p3d_lib} PRIVATE P3D_API_EXPORT)
    message("shared: true")
else()
    message("shared: false")
endif ()

if ( MSVC )
    set_target_properties( ${p3d_lib} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR} )
    set_target_properties( ${p3d_lib} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR} )
    # etc for the other available configuration types (MinSizeRel, RelWithDebInfo)
endif ( MSVC )

# ===== Tests ================================================

if (BUILD_UNIT_TESTS)

message("building tests: true")

enable_testing()

set(p3d_tests p3d_tests)
add_executable(${p3d_tests} 
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/main_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_diamond.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_fundmat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_meta.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_misc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_paths.h
    ${SOURCES}
)
target_compile_features(${p3d_tests} PRIVATE cxx_std_17)
target_compile_definitions(${p3d_tests} PRIVATE P3D_IMAGES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../_datasets")

find_package(gtest REQUIRED)
if (gtest_FOUND)
    message("google-test: found")
else()
    message(FATAL_ERROR "google-test: not found")
endif()

target_include_directories(${p3d_tests} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/src/
        ${gtest_INCLUDE_DIR}
        ${NLopt_INCLUDE_DIR}
        ${ceres-solver_INCLUDE_DIR}
        ${ceres-solver_LIB_DIRS}/../include/ceres/internal/miniglog
        ${opencv_INCLUDE_DIR}
        ${Eigen3_INCLUDE_DIR}
)

target_link_libraries(${p3d_tests}
    PUBLIC
        ${gtest_LIBS}
        ${opencv_LIBS}
        OpenMP::OpenMP_CXX
        ${Eigen3_LIBS}
    PRIVATE
        ${NLopt_LIBS}
        ${NLopt_SYSTEM_LIBS}
        ceres-solver::ceres-solver
)

add_test(
    NAME ${p3d_tests}
    COMMAND ${p3d_tests}
)

if (MSVC)
file(GLOB pollen3d_deps_dlls "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty_bin/*.dll")
file(COPY ${pollen3d_deps_dlls} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
elseif(APPLE)
file(GLOB pollen3d_deps_dlls "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty_bin/*.dylib")
file(COPY ${pollen3d_deps_dlls} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

endif(BUILD_UNIT_TESTS)

# ===== Install ==============================================

#set ( p3d_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" )
#set ( p3d_CONFIG_DEST "${CMAKE_CURRENT_SOURCE_DIR}/../cmake" )

#configure_package_config_file (
#        "${p3d_CONFIG_DEST}/p3d-config.cmake.in"
#        # Important to write in CMAKE_BINARY_DIR if you want the registry
#        # mechanism to work:
#        "${p3d_CONFIG_DEST}/p3d-config.cmake"
#        INSTALL_DESTINATION "${p3d_CONFIG_DEST}"
#        PATH_VARS p3d_INCLUDE_DIRS )

## This file is included in our template:
#export ( TARGETS ${p3d_lib} FILE "${CMAKE_BINARY_DIR}/p3dTargets.cmake" )

#export ( PACKAGE p3d )


## We redefine this variable, using this time a relative path:
#set ( p3d_INCLUDE_DIRS "src" )

## We write in the 'export' folder in order not to collide with
## the previous config file:
#configure_package_config_file (
#       "${p3d_CONFIG_DEST}/p3d-config.cmake.in"
#       "${p3d_CONFIG_DEST}/export/p3d-config.cmake.cmake"
#       INSTALL_DESTINATION "${CMAKE_CONFIG_DEST}"
#       PATH_VARS p3d_INCLUDE_DIRS )

#install (
#      EXPORT export
#      DESTINATION ${CMAKE_CONFIG_DEST} FILE "p3dTargets.cmake" )
