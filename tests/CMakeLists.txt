cmake_minimum_required(VERSION 3.1)
project(tests)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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
    BASIC_SETUP)

option(USE_OPENMP "Use OpenMP" ON)

set(CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_MODULE_PATH}
    )

# ----- Sources -----------------------------------------------

add_executable(pollen3d_tests
    ./test_paths.h
    ./test_meta.h
    ./test_misc.h
    ./test_fundmat.h
    ./test_diamond.h
    ./main_tests.cpp
    ./../lib/src/p3d/core.cpp
    ./../lib/src/p3d/core.h
    ./../lib/src/p3d/logger.cpp
    ./../lib/src/p3d/logger.h
    ./../lib/src/p3d/serialization.h
    ./../lib/src/p3d/serialization.cpp
    ./../lib/src/p3d/commands.cpp
    ./../lib/src/p3d/commands.h
    ./../lib/src/p3d/utils.cpp
    ./../lib/src/p3d/utils.h
    ./../lib/src/p3d/command_manager.h
    ./../lib/src/p3d/command_manager.cpp
    ./../lib/src/p3d/image/feature_extraction.cpp
    ./../lib/src/p3d/image/feature_extraction.h
    ./../lib/src/p3d/multiview/autocalib.cpp
    ./../lib/src/p3d/multiview/autocalib.h
    ./../lib/src/p3d/multiview/bundle_adjustment.cpp
    ./../lib/src/p3d/multiview/bundle_adjustment.h
    ./../lib/src/p3d/multiview/bundle_params.h
    ./../lib/src/p3d/stereo/dense_matching.cpp
    ./../lib/src/p3d/stereo/dense_matching.h
    ./../lib/src/p3d/stereo/fundmat.cpp
    ./../lib/src/p3d/stereo/fundmat.h
    ./../lib/src/p3d/stereo/matching.cpp
    ./../lib/src/p3d/stereo/matching.h
    ./../lib/src/p3d/stereo/rectification.cpp
    ./../lib/src/p3d/stereo/rectification.h
    ./../lib/src/p3d/tasks.h
    ./../lib/src/p3d/tasks.cpp
    ./../lib/src/p3d/data/project_settings.cpp
    ./../lib/src/p3d/data/project_settings.h
    ./../lib/src/p3d/data/affine_camera.cpp
    ./../lib/src/p3d/data/affine_camera.h
    ./../lib/src/p3d/data/image.cpp
    ./../lib/src/p3d/data/image.h
    ./../lib/src/p3d/data/image_pair.cpp
    ./../lib/src/p3d/data/image_pair.h
    ./../lib/src/p3d/data/point_cloud_container.cpp
    ./../lib/src/p3d/data/point_cloud_container.h
    ./../lib/src/p3d/data/point_cloud.cpp
    ./../lib/src/p3d/data/point_cloud.h
    ./../lib/src/p3d/project.cpp
    ./../lib/src/p3d/project.h
)

target_include_directories(pollen3d_tests SYSTEM PRIVATE ./../3rdparty)
target_include_directories(pollen3d_tests SYSTEM PRIVATE ./../lib/src)

# ----- OpenMP --------------------------------------------

if(USE_OPENMP)
    find_package(OpenMP QUIET)
    if (OPENMP_FOUND)
        message( STATUS "OpenMP - FOUND" )
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
        add_definitions(-DWITH_OPENMP)
    else()
        message( STATUS "OpenMP - not found" )
    endif()
endif()

# ----- Add Eigen --------------------------------------------

find_package(Eigen3 REQUIRED QUIET)
if (Eigen3_FOUND)
    message("Eigen3: found")
    target_include_directories(pollen3d_tests PUBLIC ${Eigen3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3: not found")
endif()

# ----- Add OpenCV --------------------------------------------

find_package(opencv REQUIRED QUIET)
if (opencv_FOUND)
    message("OpenCV: found")
    target_include_directories(pollen3d_tests PUBLIC ${opencv_INCLUDE_DIR})
else()
    message(FATAL_ERROR "OpenCV: not found")
endif()

# ----- Add NLopt --------------------------------------------

find_package(NLopt REQUIRED QUIET)
if (NLopt_FOUND)
    message("nlopt: found")
    target_include_directories(pollen3d_tests PRIVATE ${NLopt_INCLUDE_DIR})
else()
    message(FATAL_ERROR "nlopt: not found")
endif()

# ----- Add ceres --------------------------------------------

find_package(ceres-solver REQUIRED QUIET)
if (ceres-solver_FOUND)
    message("ceres-solver: found")
    target_include_directories(pollen3d_tests PRIVATE ${ceres-solver_INCLUDE_DIR})
    target_include_directories(pollen3d_tests PRIVATE ${ceres-solver_LIB_DIRS}/../include/ceres/internal/miniglog)
else()
    message(FATAL_ERROR "ceres-solver: not found")
endif()

# ----- Add testing -------------------------------------------

enable_testing()
find_package(gtest REQUIRED)
if (gtest_FOUND)
    message("google-test: found")
    target_include_directories(pollen3d_tests PUBLIC ${gtest_INCLUDE_DIR})
else()
    message(FATAL_ERROR "google-test: not found")
endif()

# -------------------------------------------------------------
# ----- Link --------------------------------------------------
# -------------------------------------------------------------

set(TARGET_LIBS ${CONAN_LIBS})
target_link_directories(pollen3d_tests PUBLIC ${CONAN_LIB_DIRS})
if(OpenMP_CXX_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} OpenMP::OpenMP_CXX)
endif()

target_link_libraries(pollen3d_tests ${TARGET_LIBS})

add_test(
    NAME pollen3d_tests
    COMMAND pollen3d_tests
)
