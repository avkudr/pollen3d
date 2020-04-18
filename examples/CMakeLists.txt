cmake_minimum_required(VERSION 3.1)
project( p3d_examples )

#==============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

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

add_custom_command(TARGET extract_features POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${P3D_LIB_DIR}/libp3d-debug.so"      # <--this is in-file
        $<TARGET_FILE_DIR:extract_features>)                 # <--this is out-file path
