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

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++17)

option(USE_OPENMP "Use OpenMP" ON)

set(CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_MODULE_PATH}
    )

include_directories(../../3rdparty/entt/src/entt/)

# ----- OpenMP

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

set(TARGET_LIBS ${CONAN_LIBS})
link_directories(${CONAN_LIB_DIRS})
if(OpenMP_CXX_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} OpenMP::OpenMP_CXX)
endif()

include_directories(${CMAKE_SOURCE_DIR}/../pollen3d_library/src)
link_directories(${CMAKE_SOURCE_DIR}/../pollen3d_library/build/lib)
target_link_libraries(pollen3d_tests pollen3d_lib_shared ${TARGET_LIBS})

add_test(
    NAME pollen3d_tests
    COMMAND pollen3d_tests
)
