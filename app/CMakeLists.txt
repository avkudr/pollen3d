cmake_minimum_required(VERSION 3.1)
project( pollen3d_app )

set(CMAKE_CXX_STANDARD 17)

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

conan_cmake_run(
    CONANFILE conanfile.txt
    BUILD missing
    BASIC_SETUP)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++17)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "compiling in debug")
    set(CMAKE_DEBUG_POSTFIX -debug)
    add_compile_definitions(POLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "compiling in release")
endif()

option(USE_OPENMP "Use OpenMP" ON)

set(ASSETS_FONTS_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/fonts CACHE STRING "Path to the folder with fonts")

# ----- Looking for modules

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cmake/)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_MODULE_PATH})

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

find_package(Eigen3 REQUIRED QUIET)
if (Eigen3_FOUND)
    message("Eigen3: found")
    include_directories(${Eigen3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3: not found")
endif()

# ----- Add opencv -------------------------------------------

find_package(opencv REQUIRED QUIET)
if (opencv_FOUND)
    message("OpenCV: found")
    include_directories(${opencv_INCLUDE_DIR})
else()
    message(FATAL_ERROR "OpenCV: not found")
endif()

# ----- Add ImGui --------------------------------------------

find_package(ImGui REQUIRED)
if (ImGui_FOUND)
    add_definitions(-DWITH_IMGUI)
    message( STATUS "ImGui: found")
else()
    message( STATUS "ImGui: not found")
endif()

# ----- Add file dialogs --------------------------------------------

set(FILE_DIALOG_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tinyfiledialogs/tinyfiledialogs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tinyfiledialogs/tinyfiledialogs.c
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/)

set_source_files_properties(
    ${FILE_DIALOG_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

set(SOURCES_GUI ${SOURCES_GUI} ${FILE_DIALOG_SOURCES})

# ----- pollen3d ---------------------------------------------

set(SOURCES_GUI
    ${SOURCES_GUI}
    ./src/app.cpp
    ./src/app.h
    ./src/common
    ./src/main_gui.cpp
    ./src/common/common.h
    ./src/common/imgui_custom.cpp
    ./src/common/imgui_custom.h
    ./src/common/palette.h
    ./src/widgets/plots.cpp
    ./src/widgets/plots.h
    ./src/widgets/widget_console.cpp
    ./src/widgets/widget_console.h
    ./src/widgets/widget_dense_matching.cpp
    ./src/widgets/widget_dense_matching.h
    ./src/widgets/widget_feature_extract.cpp
    ./src/widgets/widget_feature_extract.h
    ./src/widgets/widget.h
    ./src/widgets/widget_matching.cpp
    ./src/widgets/widget_matching.h
)

set(SOURCES_GUI
    ${SOURCES_GUI}
    ./src/viewer3d/eyecamera.cpp
    ./src/viewer3d/eyecamera.h
    ./src/viewer3d/grid_opengl.cpp
    ./src/viewer3d/grid_opengl.h
    ./src/viewer3d/pcd_view.h
    ./src/viewer3d/pcd_view_opengl.cpp
    ./src/viewer3d/pcd_view_opengl.h
    ./src/viewer3d/shader_opengl.h
    ./src/viewer3d/viewer3d.cpp
    ./src/viewer3d/viewer3d.h
    ./src/viewer3d/viewer3d_opengl.cpp
    ./src/viewer3d/viewer3d_opengl.h
)

set(RES_FILES "")
if (WIN32)
    set(RES_FILES
        ${RES_FILES}
        ./src/pollen3d.ico
    )
endif (WIN32)

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

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty)

set(TARGET_LIBS ${TARGET_LIBS} ${ImGui_LIBS} ${CONAN_LIBS})
if(Eigen_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} Eigen3::Eigen)
endif()
if(OpenMP_CXX_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} OpenMP::OpenMP_CXX)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Compiling in Debug")
    add_compile_definitions(POLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Compiling in Release")
endif()

add_executable( ${PROJECT_NAME} ./src/main_gui.cpp ${SOURCES} ${SOURCES_GUI} ${RES_FILES} ./src/pollen3d.rc)

# ----- add library

set(P3D_LIB_DIR ${GLOBAL_BINARY_DIR}/lib)
set(P3D_INCLUDE_DIR ${GLOBAL_SOURCE_DIR}/lib/src)
target_include_directories(${PROJECT_NAME} PRIVATE ${P3D_INCLUDE_DIR})

target_link_directories(${PROJECT_NAME} PRIVATE ${P3D_LIB_DIR} )
target_link_libraries( ${PROJECT_NAME} ${TARGET_LIBS} p3d)


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${P3D_LIB_DIR}/libp3d-debug.so"      # <--this is in-file
        $<TARGET_FILE_DIR:extract_features>)                 # <--this is out-file path


# ===========================================================================================================

set_target_properties( ${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../_datasets/brassica DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)

execute_process(COMMAND ln -s /usr/local/bin/pollen3d_dir/pollen3d pollen3d)
install(PROGRAMS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/pollen3d DESTINATION /usr/local/bin/pollen3d_dir)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION /usr/local/bin/pollen3d_dir)
install(FILES ${CMAKE_BINARY_DIR}/pollen3d DESTINATION /usr/local/bin/)

# ***********************************************************************
# generate DEB package

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

    string(TIMESTAMP DATETIME_NOW %Y%m%d%H%M UTC)

    if(NOT DEFINED ARCH)

        execute_process(
            COMMAND "/usr/bin/dpkg" "--print-architecture"
            RESULT_VARIABLE RESULT
            OUTPUT_VARIABLE ARCH
        )

        string(STRIP "${ARCH}" ARCH)

    endif()

    if("${ARCH}" STREQUAL "")
        set(ARCH "amd64")
    endif()

    set(VERSION_MAJOR "0")
    set(VERSION_MINOR "9")
    set(VERSION_PATCH "${DATETIME_NOW}")

    set(CPACK_GENERATOR "DEB")
    SET(CPACK_PACKAGE_VERSION_MAJOR "${VERSION_MAJOR}")
    SET(CPACK_PACKAGE_VERSION_MINOR "${VERSION_MINOR}")
    SET(CPACK_PACKAGE_VERSION_PATCH "${VERSION_PATCH}")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCH})
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
    SET(CPACK_PACKAGE_VENDOR "AVK")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "3D recostruction in Scanning Electron Microscope (SEM)")
    set(CPACK_PACKAGE_DESCRIPTION "3D recostruction in Scanning Electron Microscope (SEM)")
    set(CPACK_PACKAGE_CONTACT "Andrey Kudryavtsev, andrey.kudryavtsev at gmail.com")

    set(PROJECT_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}")

    set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}_${PROJECT_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

    include(CPack)

endif()

