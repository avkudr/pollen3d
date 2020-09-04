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

set(P3D_LIB_NAME p3d)
set(P3D_LIB_DIR ${GLOBAL_BINARY_DIR}/lib)
set(P3D_INCLUDE_DIR ${GLOBAL_SOURCE_DIR}/lib/src)
set(P3D_LIB_POSTFIX "")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(P3D_LIB_POSTFIX -debug)
endif()

if (WIN32)
    set(FULL_PATH_P3D_LIB ${P3D_LIB_DIR}/${P3D_LIB_NAME}${P3D_LIB_POSTFIX}.dll)
elseif(APPLE)
    set(FULL_PATH_P3D_LIB ${P3D_LIB_DIR}/${P3D_LIB_NAME}${P3D_LIB_POSTFIX}.dylib)
elseif(UNIX)
    set(FULL_PATH_P3D_LIB ${P3D_LIB_DIR}/lib${P3D_LIB_NAME}${P3D_LIB_POSTFIX}.so)
endif()

function(add_p3d_example arg1)
    add_executable(${arg1} ./${arg1}.cpp)
    target_compile_features(${arg1} PUBLIC cxx_std_17)
    target_compile_definitions(${arg1} PRIVATE P3D_API_IMPORT)
    target_include_directories(${arg1} SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/)
    target_compile_definitions(${arg1} PRIVATE P3D_IMAGES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../_datasets")

    # ----- add library

    target_include_directories(${arg1} PRIVATE ${P3D_INCLUDE_DIR})
    link_directories(${arg1} PRIVATE ${P3D_LIB_DIR} )
    target_link_libraries(${arg1} PRIVATE p3d )

    add_custom_command(TARGET ${arg1} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            ${FULL_PATH_P3D_LIB}      # <--this is in-file
            $<TARGET_FILE_DIR:${arg1}>)
endfunction()

file(GLOB p3d_LIBS "${P3D_LIB_DIR}/*.so")
file(COPY ${p3d_LIBS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
file(GLOB p3d_LIBS "${P3D_LIB_DIR}/*.dylib")
file(COPY ${p3d_LIBS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
file(GLOB p3d_LIBS "${P3D_LIB_DIR}/*.dll")
file(COPY ${p3d_LIBS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)

add_p3d_example(extract_features)
add_p3d_example(reconstruction_batch)
