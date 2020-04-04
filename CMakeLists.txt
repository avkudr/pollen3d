cmake_minimum_required(VERSION 3.1)
project( pollen3d )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

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

#include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
#conan_basic_setup()
#include_directories(${CONAN_INCLUDE_DIRS})
#link_directories(${CONAN_LIB_DIRS})

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Compiling in Debug")
    add_compile_definitions(POLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Compiling in Release")
endif()

OPTION(BUILD_UNIT_TESTS "Build unit tests" ON)
OPTION(USE_OPENMP "Use OpenMP" ON)

set(ASSETS_FONTS_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/fonts CACHE STRING "Path to the folder with fonts")

# ----- Looking for modules

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cmake/)
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

set(SOURCES "")
set(SOURCES_GUI "")

# ----- Threads

#find_package (Threads)

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

set( EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen-3.3.7)
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories ( "${EIGEN3_INCLUDE_DIR}" )

# ----- Add opencv -------------------------------------------

find_package(opencv 3.4.0 REQUIRED)
if (opencv_FOUND)
    link_directories(${CONAN_INCLUDE_DIRS_OPENCV}/../staticlib)
endif()

# ----- Add Ceres --------------------------------------------

find_package(ceres-solver REQUIRED)
if (ceres-solver_FOUND)
    include_directories(${ceres-solver_INCLUDE_DIRS})
    include_directories(${ceres-solver_LIB_DIRS}/../include/ceres/internal/miniglog)
endif()

# ----- Add NLopt --------------------------------------------

find_package(NLopt REQUIRED)
if (NLOPT_FOUND)
    include_directories(${NLOPT_INCLUDE_DIRS})
endif()

# ----- Add ImGui --------------------------------------------

find_package(ImGui REQUIRED)
if (ImGui_FOUND)
    add_definitions(-DWITH_IMGUI)
    message( STATUS "ImGui - FOUND")
else()
    message( STATUS "ImGui - not found")
endif()

# ----- Add file dialogs --------------------------------------------

set(FILE_DIALOG_SOURCES
    ./3rdparty/tinyfiledialogs/tinyfiledialogs.h
    ./3rdparty/tinyfiledialogs/tinyfiledialogs.c
)

set_source_files_properties(
    ${FILE_DIALOG_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

set(SOURCES_GUI ${SOURCES_GUI} ${FILE_DIALOG_SOURCES})

include_directories(./3rdparty/)

# ----- pollen3d ---------------------------------------------

file(GLOB_RECURSE SOURCES ./src/lib/*.*)

set(SOURCES_GUI
    ${SOURCES_GUI}
    ./src/app.h
    ./src/app.cpp
    ./src/gui/common.h
    ./src/gui/palette.h
    ./src/gui/widget.h
    ./src/gui/imgui_custom.h
    ./src/gui/imgui_custom.cpp
    ./src/gui/widget_console.h
    ./src/gui/widget_console.cpp
    ./src/gui/widget_feature_extract.h
    ./src/gui/widget_feature_extract.cpp
    ./src/gui/widget_matching.h
    ./src/gui/widget_matching.cpp
    ./src/gui/widget_dense_matching.h
    ./src/gui/widget_dense_matching.cpp
)

set(SOURCES_GUI
    ${SOURCES_GUI}
    ./src/viewer3d/viewer3d.h
    ./src/viewer3d/viewer3d.cpp
    ./src/viewer3d/eyecamera.h
    ./src/viewer3d/eyecamera.cpp
    ./src/viewer3d/viewer3d_opengl.h
    ./src/viewer3d/viewer3d_opengl.cpp
    ./src/viewer3d/shader_opengl.h
    ./src/viewer3d/grid_opengl.h
    ./src/viewer3d/grid_opengl.cpp
)

set(RES_FILES "")
if(POLLEN3D_OPENGL)
    message("OpenGL - FOUND")
    set(SOURCES_GUI
        ${SOURCES_GUI}
        ./src/app_opengl.cpp
        ./src/app_opengl.h
    )
    file(GLOB RES_FILES ./assets/*.vs ./assets/*.fs)

elseif(POLLEN3D_VULKAN)
    error( "OpenGL not found" )
    error( "Vulkan not found" )
else()

endif()

include_directories(./3rdparty/entt/src/entt/)
include_directories(./src/lib)

set(TARGET_LIBS ${TARGET_LIBS} ${ImGui_LIBS} ${CONAN_LIBS})
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

if (BUILD_UNIT_TESTS)
    message("Tests: YES")
    include(${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
else()
    message("Tests: NO")
endif()

add_executable( ${PROJECT_NAME} ./src/main_gui.cpp ${SOURCES} ${SOURCES_GUI} ${RES_FILES})
target_link_libraries( ${PROJECT_NAME} ${TARGET_LIBS})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR}/bin)
