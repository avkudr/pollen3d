cmake_minimum_required(VERSION 3.1)
project( pollen3d_library )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "compiling in debug")
    set(CMAKE_DEBUG_POSTFIX _d)
    add_compile_definitions(POLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "compiling in release")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

option(USE_OPENMP "Use OpenMP" ON)

set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR} ${CMAKE_MODULE_PATH})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)

include_directories(../../3rdparty/entt/src/entt/)

# ----- Add Eigen --------------------------------------------

find_package(Eigen3 REQUIRED QUIET)
if (Eigen3_FOUND)
    message("Eigen3: found")
    include_directories(${Eigen3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3: not found")
endif()

# ----- Add OpenCV --------------------------------------------

find_package(opencv REQUIRED QUIET)
if (opencv_FOUND)
    message("OpenCV: found")
    include_directories(${opencv_INCLUDE_DIR})
else()
    message(FATAL_ERROR "OpenCV: not found")
endif()

# ----- Add NLopt --------------------------------------------

find_package(NLopt REQUIRED QUIET)
if (NLopt_FOUND)
    message("nlopt: found")
    include_directories(${NLopt_INCLUDE_DIR})
else()
    message(FATAL_ERROR "nlopt: not found")
endif()

# ----- Add ceres --------------------------------------------

find_package(ceres-solver REQUIRED QUIET)
if (ceres-solver_FOUND)
    message("ceres-solver: found")
    include_directories(${ceres-solver_INCLUDE_DIR})
    include_directories(${ceres-solver_LIB_DIRS}/../include/ceres/internal/miniglog)
else()
    message(FATAL_ERROR "ceres-solver: not found")
endif()

# ----- Sources ----------------------------------------------

include_directories(./src/)

add_library(pollen3d_library STATIC
    ./src/p3d/commands.cpp
    ./src/p3d/commands.h
    ./src/p3d/core.cpp
    ./src/p3d/core.h
    ./src/p3d/data
    ./src/p3d/logger.cpp
    ./src/p3d/logger.h
    ./src/p3d/multiview
    ./src/p3d/project_manager.cpp
    ./src/p3d/project_manager.h
    ./src/p3d/serialization.cpp
    ./src/p3d/serialization.h
    ./src/p3d/stereo
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
    ./src/p3d/data/project_data.cpp
    ./src/p3d/data/project_data.h
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

set_target_properties(pollen3d_library PROPERTIES OUTPUT_NAME p3d)


# target_compile_features(p3d PUBLIC cxx_std_17)
# target_include_directories(p3d PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#     $<INSTALL_INTERFACE:include>
# )
# add_library(pollen3d::lib ALIAS p3d)

# install(TARGETS p3d EXPORT p3d
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin
#         INCLUDES DESTINATION include)

# install(EXPORT p3d
#         DESTINATION lib/cmake/p3d
#         FILE p3dTargets.cmake
#         NAMESPACE pollen3d::)

# install(DIRECTORY src/p3d
#         DESTINATION include
#         FILES_MATCHING PATTERN "*.h*"
#         )

# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(p3dConfigVersion.cmake
#     VERSION 0.9.0
#     COMPATIBILITY SameMajorVersion
# )
# install(FILES p3dConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/p3dConfigVersion.cmake
# DESTINATION lib/cmake/p3d)