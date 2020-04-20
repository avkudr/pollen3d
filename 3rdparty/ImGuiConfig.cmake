set(ImGui_DIR ${CMAKE_CURRENT_LIST_DIR}/imgui)
set(ImGui_FOUND false)
set(ImGui_LIBS "")

include_directories(
    SYSTEM
    ${ImGui_DIR}
    )

set(SOURCES_GUI
    ${SOURCES_GUI}
    ${ImGui_DIR}/imgui.h
    ${ImGui_DIR}/imgui_internal.h
    ${ImGui_DIR}/imstb_rectpack.h
    ${ImGui_DIR}/imstb_textedit.h
    ${ImGui_DIR}/imstb_truetype.h
    ${ImGui_DIR}/imconfig.h
    ${ImGui_DIR}/imgui.cpp
    ${ImGui_DIR}/imgui_demo.cpp
    ${ImGui_DIR}/imgui_draw.cpp
    ${ImGui_DIR}/imgui_widgets.cpp
    )

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL QUIET)
if (OPENGL_FOUND)
    add_definitions(-DWITH_IMGUI)
    add_definitions(-DWITH_OPENGL)

    message( STATUS "OpenGL - FOUND")
else()
    message( FATAL_ERROR "OpenGL - not found")
endif()

find_package(glfw REQUIRED)
find_package(glad REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR})

set(ImGui_FOUND true)
set(ImGui_LIBS ${OPENGL_LIBRARIES} ${glfw_LIBS} ${glad_LIBS})
set(ImGui_INCLUDE_DIR ${OPENGL_INCLUDE_DIR} ${glfw_INCLUDE_DIR} ${glad_INCLUDE_DIR})

add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLAD)

if (ImGui_FOUND)
    set(SOURCES_GUI
        ${SOURCES_GUI}
        ${ImGui_DIR}/examples/imgui_impl_glfw.h
        ${ImGui_DIR}/examples/imgui_impl_glfw.cpp
        ${ImGui_DIR}/examples/imgui_impl_opengl3.h
        ${ImGui_DIR}/examples/imgui_impl_opengl3.cpp
        )
    set(POLLEN3D_OPENGL ON)
    add_definitions(-DPOLLEN3D_OPENGL)
else()
    message("Can't find any ImGui backend")
endif()

