cmake_minimum_required(VERSION 3.1)

#
# Read project version
#

set(POLLEN3D_VERSION_REGEX "#define POLLEN3D_VERSION_.*[ \t]+(.+)")
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/src/version.h" POLLEN3D_VERSION REGEX ${POLLEN3D_VERSION_REGEX})
list(TRANSFORM POLLEN3D_VERSION REPLACE ${POLLEN3D_VERSION_REGEX} "\\1")
list(GET POLLEN3D_VERSION 0 POLLEN3D_VERSION_MAJOR)
list(GET POLLEN3D_VERSION 1 POLLEN3D_VERSION_MINOR)
list(GET POLLEN3D_VERSION 2 POLLEN3D_VERSION_PATCH)
string(JOIN "." POLLEN3D_VERSION ${POLLEN3D_VERSION})

project(
    pollen3d
    VERSION ${POLLEN3D_VERSION}
    DESCRIPTION "3D reconstruction in Scanning Electron Microscope"
    HOMEPAGE_URL "https://github.com/avkudr/pollen3d"
    LANGUAGES CXX
    )

message("*")
message("* ${PROJECT_NAME} v${PROJECT_VERSION} (${CMAKE_BUILD_TYPE})")
message("* Copyright (c) 2020 Andrey Kudryavtsev <andrey.kudryavtsev.24@gmail.com>")
message("*")

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_DEBUG_POSTFIX -debug)
    add_definitions(-DPOLLEN3D_DEBUG)
endif()

option(USE_OPENMP "Use OpenMP" ON)

set(ASSETS_FONTS_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/fonts CACHE STRING "Path to the folder with fonts")

# ----- Looking for modules

set(CMAKE_PREFIX_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty 
    ${GLOBAL_3RDPARTY_BIN}
    ${CMAKE_PREFIX_PATH}
)

set(CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty 
    ${GLOBAL_3RDPARTY_BIN}
    ${CMAKE_MODULE_PATH}
)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/portable-file-dialogs/portable-file-dialogs.h
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
    ./src/version.h
    ./src/common/common.h
    ./src/common/common.cpp
    ./src/common/imgui_custom.cpp
    ./src/common/imgui_custom.h
    ./src/common/palette.h
    ./src/common/app_state.h
    ./src/common/app_state.cpp
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
    ./src/widgets/heavy_task.h
    ./src/widgets/heavy_task.cpp
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
    ./src/viewer3d/camera_view.h
    ./src/viewer3d/camera_view_opengl.h
    ./src/viewer3d/camera_view_opengl.cpp
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

set(TARGET_LIBS "")
if(ImGui_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} ${ImGui_LIBS})
endif()
if(Eigen_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} Eigen3::Eigen)
endif()
if(OpenMP_CXX_FOUND)
    set(TARGET_LIBS ${TARGET_LIBS} OpenMP::OpenMP_CXX)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Compiling in Debug")
    add_definitions(-DPOLLEN3D_DEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Compiling in Release")
endif()

add_executable( ${PROJECT_NAME} ./src/main_gui.cpp ${SOURCES} ${SOURCES_GUI} ${RES_FILES} ./src/pollen3d.rc)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE P3D_API_IMPORT)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty)
target_include_directories(${PROJECT_NAME} PUBLIC ${ImGui_INCLUDE_DIR})

# ----- link p3d library ----------------------------------------------------------------------------------

set(P3D_LIB_NAME p3d)
set(P3D_LIB_DIR ${GLOBAL_BINARY_DIR}/lib)
set(P3D_INCLUDE_DIR ${GLOBAL_SOURCE_DIR}/lib/src)

target_include_directories(${PROJECT_NAME} PRIVATE ${P3D_INCLUDE_DIR})
link_directories(${P3D_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE ${TARGET_LIBS} ${P3D_LIB_NAME} )

set(P3D_LIB_POSTFIX "")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(P3D_LIB_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif()

if (WIN32)
    set(FULL_PATH_P3D_LIB ${P3D_LIB_DIR}/${P3D_LIB_NAME}${P3D_LIB_POSTFIX}.dll)
elseif(APPLE)
    set(FULL_PATH_P3D_LIB ${P3D_LIB_DIR}/${P3D_LIB_NAME}${P3D_LIB_POSTFIX}.dylib)
elseif(UNIX)
    set(FULL_PATH_P3D_LIB ${P3D_LIB_DIR}/lib${P3D_LIB_NAME}${P3D_LIB_POSTFIX}.so)
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        ${FULL_PATH_P3D_LIB}      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)

# ===========================================================================================================

set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/lib )
set_target_properties( ${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/lib )
set_target_properties( ${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
set_target_properties( ${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/lib )
set_target_properties( ${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/lib )
set_target_properties( ${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
set_target_properties( ${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/bin )
set_target_properties( ${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/bin )
# etc for the other available configuration types (MinSizeRel, RelWithDebInfo)

# ==== copy files needed for execution =======================================================================

if (MSVC)
file(GLOB pollen3d_deps_dlls "${GLOBAL_3RDPARTY_BIN}/*.dll")
file(COPY ${pollen3d_deps_dlls} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
elseif(APPLE)
file(GLOB pollen3d_deps_dlls "${GLOBAL_3RDPARTY_BIN}/*.dylib")
file(COPY ${pollen3d_deps_dlls} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
endif()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../_datasets/brassica DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)

file(WRITE ${CMAKE_BINARY_DIR}/tmp_/pollen3d_script
    "#!/bin/sh\n"
    "export LD_LIBRARY_PATH=/usr/local/bin/pollen3d_dir:\$LD_LIBRARY_PATH\n"
    "/usr/local/bin/pollen3d_dir/pollen3d\n"
    )
file(COPY ${CMAKE_BINARY_DIR}/tmp_/pollen3d_script DESTINATION ${CMAKE_BINARY_DIR}/ FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE)
file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/tmp_/pollen3d_script)

execute_process(COMMAND ln -sf /usr/local/bin/pollen3d_dir/pollen3d_script pollen3d)
install(FILES ${CMAKE_BINARY_DIR}/pollen3d DESTINATION /usr/local/bin/)

install(PROGRAMS ${CMAKE_BINARY_DIR}/pollen3d_script DESTINATION /usr/local/bin/pollen3d_dir 
    PERMISSIONS 
        OWNER_READ OWNER_WRITE OWNER_EXECUTE 
        GROUP_READ GROUP_WRITE GROUP_EXECUTE 
        WORLD_READ WORLD_WRITE WORLD_EXECUTE
)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/bin/pollen3d DESTINATION /usr/local/bin/pollen3d_dir)
install(FILES ${P3D_LIB_DIR}/libp3d.so DESTINATION /usr/local/bin/pollen3d_dir)

# ===============================================================================================

file(GLOB opencv_so "${opencv_LIB_DIRS}/*.so.3.4")
install(FILES ${opencv_so} DESTINATION /usr/local/bin/pollen3d_dir)

install( TARGETS ${PROJECT_NAME} RUNTIME DESTINATION /usr/local/bin/pollen3d_dir )
set_target_properties( ${PROJECT_NAME} PROPERTIES INSTALL_RPATH "/usr/local/bin/pollen3d_dir" )

# ===============================================================================================

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION /usr/local/bin/pollen3d_dir)
install(FILES ${CMAKE_BINARY_DIR}/pollen3d DESTINATION /usr/local/bin/)

# ***********************************************************************
# generate DEB package

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

    string(TIMESTAMP DATETIME_NOW %Y-%m-%d UTC)

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

    set(CPACK_GENERATOR "DEB")
    set(CPACK_PACKAGE_VERSION_MAJOR "${POLLEN3D_VERSION_MAJOR}")
    set(CPACK_PACKAGE_VERSION_MINOR "${POLLEN3D_VERSION_MINOR}")
    set(CPACK_PACKAGE_VERSION_PATCH "${POLLEN3D_VERSION_PATCH}")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${ARCH})
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
    set(CPACK_PACKAGE_VENDOR "AVK")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "3D reconstruction in Scanning Electron Microscope (SEM)")
    set(CPACK_PACKAGE_DESCRIPTION "3D reconstruction in Scanning Electron Microscope (SEM)")
    set(CPACK_PACKAGE_CONTACT "Andrey Kudryavtsev, andrey.kudryavtsev.24 at gmail.com")

    set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}-${DATETIME_NOW}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

    include(CPack)

endif()

