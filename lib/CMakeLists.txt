cmake_minimum_required(VERSION 3.1)
project( p3d )

#==============================================================================

option (BUILD_SHARED_LIBS "Build p3d as a shared library" ON)

#==============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Download automatically, you can also just copy the conan.cmake file
if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/conan.cmake")
   message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
   file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake"
                  "${CMAKE_CURRENT_BINARY_DIR}/conan.cmake")
endif()

include(${CMAKE_CURRENT_BINARY_DIR}/conan.cmake)

conan_check()
conan_add_remote(NAME bincrafters INDEX 1
            URL https://api.bintray.com/conan/bincrafters/public-conan)
conan_cmake_run(
    CONANFILE conanfile.txt
    BUILD missing
    BASIC_SETUP NO_OUTPUT_DIRS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "compiling in debug")
    set(CMAKE_DEBUG_POSTFIX -debug)
    add_compile_definitions(POLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "compiling in release")
endif()

option(USE_OPENMP "Use OpenMP" ON)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_MODULE_PATH})

# s111et(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/)
# s111et(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/)

# ----- Sources ----------------------------------------------

set(SOURCES
    ./src/p3d/commands.cpp
    ./src/p3d/commands.h
    ./src/p3d/command_manager.h
    ./src/p3d/command_manager.cpp
    ./src/p3d/tasks.h
    ./src/p3d/tasks.cpp
    ./src/p3d/core.cpp
    ./src/p3d/core.h
    ./src/p3d/logger.cpp
    ./src/p3d/logger.h
    ./src/p3d/serialization.cpp
    ./src/p3d/serialization.h
    ./src/p3d/utils.cpp
    ./src/p3d/utils.h
    ./src/p3d/data/affine_camera.cpp
    ./src/p3d/data/affine_camera.h
    ./src/p3d/data/image.cpp
    ./src/p3d/data/image.h
    ./src/p3d/data/image_pair.cpp
    ./src/p3d/data/image_pair.h
    ./src/p3d/data/point_cloud_container.cpp
    ./src/p3d/data/point_cloud_container.h
    ./src/p3d/data/point_cloud.cpp
    ./src/p3d/data/point_cloud.h
    ./src/p3d/project.cpp
    ./src/p3d/project.h
    ./src/p3d/data/project_settings.cpp
    ./src/p3d/data/project_settings.h
    ./src/p3d/multiview/autocalib.cpp
    ./src/p3d/multiview/autocalib.h
    ./src/p3d/multiview/bundle_adjustment.cpp
    ./src/p3d/multiview/bundle_adjustment.h
    ./src/p3d/multiview/bundle_params.h
    ./src/p3d/stereo/dense_matching.cpp
    ./src/p3d/stereo/dense_matching.h
    ./src/p3d/stereo/fundmat.cpp
    ./src/p3d/stereo/fundmat.h
    ./src/p3d/stereo/matching.cpp
    ./src/p3d/stereo/matching.h
    ./src/p3d/stereo/rectification.cpp
    ./src/p3d/stereo/rectification.h
)

set (p3d_lib p3d)
add_library (${p3d_lib} ${SOURCES})

target_include_directories(${p3d_lib} SYSTEM PRIVATE ./../3rdparty/)
target_include_directories(${p3d_lib} SYSTEM PRIVATE ./src/)

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

target_link_libraries(${p3d_lib}
    PUBLIC
        ${opencv_LIBS}
    PRIVATE
        ${NLopt_LIBS}
        ${ceres-solver_LIBS}
    )

if (BUILD_SHARED_LIBS)
    target_compile_definitions (${p3d_lib} PRIVATE P3D_API_EXPORT)
    message("shared: true")
else()
    message("shared: false")
endif ()
