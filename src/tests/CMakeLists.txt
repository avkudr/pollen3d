cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CONAN_SYSTEM_INCLUDES ON)
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/v0.14/conan.cmake"
            "${CMAKE_BINARY_DIR}/conan.cmake")
endif()
include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_run(
CONANFILE conanfile.txt
BASIC_SETUP
BUILD missing)

conan_check()

set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

include_directories(../../3rdparty/entt/src/entt/)

# ----- Add testing -------------------------------------------

enable_testing()
find_package(gtest REQUIRED)
if (gtest_FOUND)
    message("GoogleTest - FOUND")
endif()
include_directories(${GTEST_INCLUDE_DIR})

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


include_directories(${CMAKE_SOURCE_DIR}/../pollen3d_library/src)
link_directories(${CMAKE_SOURCE_DIR}/../pollen3d_library/build/lib/)
target_link_libraries(pollen3d_tests p3d ${CONAN_LIBS})

add_test(
    NAME pollen3d_tests
    COMMAND pollen3d_tests
)
