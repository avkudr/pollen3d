cmake_minimum_required(VERSION 3.1)
project( pollen3d_all )

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Prevented in-tree built. Please create a build directory outside of the source code and call cmake from there. Thank you.")
endif()

option(BUILD_UNIT_TESTS "Build unit tests" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)

set(CONAN_CMAKE_SILENT_OUTPUT 1)

set(GLOBAL_3RDPARTY_BIN "")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(GLOBAL_3RDPARTY_BIN ${CMAKE_SOURCE_DIR}/3rdparty_debug)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(GLOBAL_3RDPARTY_BIN ${CMAKE_SOURCE_DIR}/3rdparty_release)
endif()

set(GLOBAL_BINARY_DIR ${CMAKE_BINARY_DIR})
set(GLOBAL_SOURCE_DIR ${CMAKE_SOURCE_DIR})

add_subdirectory(./lib)
add_subdirectory(./app)

if(BUILD_EXAMPLES)
    add_subdirectory(./examples)
endif()
