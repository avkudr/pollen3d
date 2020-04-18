cmake_minimum_required(VERSION 3.1)
project( p3d_examples )

#==============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
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
conan_add_remote(NAME camposs INDEX 2
            URL https://conan.campar.in.tum.de/api/conan/conan-camposs)

conan_cmake_run(
    CONANFILE ../conanfile.txt
    BUILD missing
    BASIC_SETUP)

# ----- Sources ----------------------------------------------

add_executable(extract_features ./extract_features.cpp)
target_compile_features(extract_features PUBLIC cxx_std_17)
target_compile_definitions(extract_features PRIVATE P3D_API_IMPORT)
target_include_directories(extract_features SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/)
target_compile_definitions(extract_features PRIVATE P3D_IMAGES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../_datasets")

# ----- add library

set(P3D_LIB_DIR ${GLOBAL_BINARY_DIR}/lib)
set(P3D_INCLUDE_DIR ${GLOBAL_SOURCE_DIR}/lib/src)

target_include_directories(extract_features PRIVATE ${P3D_INCLUDE_DIR})
target_link_directories(extract_features PRIVATE ${P3D_LIB_DIR} )
target_link_libraries(extract_features PRIVATE p3d )

file(GLOB p3d_LIBS "${P3D_LIB_DIR}/*.so")
file(COPY ${p3d_LIBS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
file(GLOB p3d_LIBS "${P3D_LIB_DIR}/*.dylib")
file(COPY ${p3d_LIBS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
file(GLOB p3d_LIBS "${P3D_LIB_DIR}/*.dll")
file(COPY ${p3d_LIBS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)

