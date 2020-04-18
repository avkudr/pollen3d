cmake_minimum_required(VERSION 3.1)
project( pollen3d_all )

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Prevented in-tree built. Please create a build directory outside of the source code and call cmake from there. Thank you.")
endif()

option(BUILD_UNIT_TESTS "Build unit tests" ON)

set(GLOBAL_BINARY_DIR ${CMAKE_BINARY_DIR})
set(GLOBAL_SOURCE_DIR ${CMAKE_SOURCE_DIR})

add_subdirectory(./lib)
add_subdirectory(./examples)
#add_subdirectory(./tests)
add_subdirectory(./app)

#add_subdirectory(./src/tests)
