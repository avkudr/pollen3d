cmake_minimum_required(VERSION 3.1)
project( pollen3d )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cmake/)

#set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Compiling in Debug")
    add_compile_definitions(POLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message("Compiling in Release")
endif()

option(BUILD_UNIT_TESTS "Build unit tests" ON)
OPTION(USE_OPENMP "Use OpenMP" ON)

# ----- Threads

#find_package (Threads)

# ----- OpenMP

if(USE_OPENMP)
    find_package(OpenMP QUIET)
    if (OPENMP_FOUND)
        message( "OpenMP - FOUND" )
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
        add_definitions(-DWITH_OPENMP)
    else()
        message( "OpenMP - not found" )
    endif()
endif()

# ----- Add Eigen --------------------------------------------

find_package (Eigen3 REQUIRED NO_MODULE)

# ----- Add OpenCV -------------------------------------------

find_package(OpenCV 3.0 QUIET)
if (OpenCV_FOUND)
    include_directories(
        SYSTEM
        ${OpenCV_INCLUDE_DIRS}
    )
else()
    message( "OpenCV not found" )
    error( "OpenCV not found" )
endif()

# ----- Add ImGui --------------------------------------------

find_package(ImGui REQUIRED)
if (ImGui_FOUND)
    add_definitions(-DWITH_IMMGUI)
    message("ImGui - FOUND")
else()
    message("ImGui - not found")
endif()

# ----- Add file dialogs --------------------------------------------

set(SOURCES
    ${SOURCES}
    ./3rdparty/tinyfiledialogs/tinyfiledialogs.h
    ./3rdparty/tinyfiledialogs/tinyfiledialogs.c
)

include_directories(./3rdparty/)

# ----- pollen3d ---------------------------------------------

set(SOURCES
    ${SOURCES}
    ./src/p3d/app.h
    ./src/p3d/app.cpp
    ./src/p3d/console_logger.h
    ./src/p3d/console_logger.cpp
    ./src/p3d/commands.h
    ./src/p3d/commands.cpp
)

set(SOURCES
    ${SOURCES}
    ./src/p3d/project_manager.h
    ./src/p3d/project_manager.cpp
    ./src/p3d/core/core.cpp
    ./src/p3d/core/core.h
    ./src/p3d/core/serialization.h
    ./src/p3d/core/serialization.cpp
    ./src/p3d/core/densematcher.cpp
    ./src/p3d/core/densematcher.h
    ./src/p3d/core/fundmat.cpp
    ./src/p3d/core/fundmat.h
    ./src/p3d/core/imageproperties.cpp
    ./src/p3d/core/imageproperties.h
    ./src/p3d/core/rectification.cpp
    ./src/p3d/core/rectification.h
    ./src/p3d/core/robust_estim.hpp
    ./src/p3d/core/utils.cpp
    ./src/p3d/core/utils.h
    ./src/p3d/data/project_data.cpp
    ./src/p3d/data/project_data.h
    ./src/p3d/data/project_settings.cpp
    ./src/p3d/data/project_settings.h
    ./src/p3d/data/image.cpp
    ./src/p3d/data/image.h
    ./src/p3d/data/image_pair.cpp
    ./src/p3d/data/image_pair.h
)
if(POLLEN3D_OPENGL)
    message("OpenGL - FOUND")
    set(SOURCES
        ${SOURCES}
        ./src/p3d/app_opengl.cpp
        ./src/p3d/app_opengl.h
    )
elseif(POLLEN3D_VULKAN)
    error( "OpenGL not found" )
    error( "Vulkan not found" )
else()

endif()

include_directories(./3rdparty/meta/src/)
include_directories(./src/)

if (BUILD_UNIT_TESTS)
    message("Tests: YES")
    include(${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
else()
    message("Tests: NO")
endif()

add_executable( ${PROJECT_NAME} ./src/p3d/main_gui.cpp ${SOURCES})
target_link_libraries( ${PROJECT_NAME} ${ImGui_LIBS} ${OpenCV_LIBS} Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

