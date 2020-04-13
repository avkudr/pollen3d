cmake_minimum_required(VERSION 2.6)
project(tests)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_DEBUG_POSTFIX d)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

option(USE_OPENMP "Use OpenMP" ON)

set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR} ${CMAKE_MODULE_PATH})

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

# ----- Add testing -------------------------------------------

enable_testing()
find_package(gtest REQUIRED)
if (gtest_FOUND)
    message("google-test: found")
    include_directories(${gtest_INCLUDE_DIR})
else()
    message(FATAL_ERROR "google-test: not found")
endif()

# ----- Sources -----------------------------------------------

add_executable(pollen3d_tests
    ./test_paths.h
    ./test_meta.h
    ./test_misc.h
    ./test_fundmat.h
    ./test_diamond.h
    ./main_tests.cpp
    ${SOURCES}
)

set(TARGET_LIBS "")
if(Eigen_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} Eigen3::Eigen)
endif()
if(Ceres_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} ${CERES_LIBRARIES})
endif()
if(NLOPT_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} ${NLOPT_LIBRARIES})
endif()
if(OpenMP_CXX_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} OpenMP::OpenMP_CXX)
endif()

include_directories(${CMAKE_SOURCE_DIR}/../pollen3d_library/src)
link_directories(${CMAKE_SOURCE_DIR}/../pollen3d_library/build/lib)
target_link_libraries(pollen3d_tests p3d ${TARGET_LIBS} ${gtest_LIBS})

add_test(
    NAME pollen3d_tests
    COMMAND pollen3d_tests
)
