set(ImGui_DIR ${CMAKE_CURRENT_LIST_DIR}/../imgui)
set(ImGui_FOUND false)
set(ImGui_LIBS "")

include_directories(
    SYSTEM
    ${ImGui_DIR}
)

set(SOURCES
    ${SOURCES}
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
find_package(glfw3 QUIET)
find_package(GLEW QUIET)
message(OPENGL_FOUND: ${glfw3_FOUND})
message(glfw3_FOUND: ${glfw3_FOUND})
message(GLEW_FOUND: ${GLEW_FOUND})

if (glfw3_FOUND AND OPENGL_FOUND AND GLEW_FOUND)
    if(APPLE)
        find_path(GLEW_INCLUDE_DIR GL/glew.h)
    endif()
    set(ImGui_FOUND true)
    include_directories(${GLEW_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
    set(ImGui_LIBS
        ${OPENGL_LIBRARIES}
        glfw
        GLEW
    )

    add_compile_definitions(IMGUI_IMPL_OPENGL_LOADER_GLEW)
    set(SOURCES
        ${SOURCES}
        ${ImGui_DIR}/examples/imgui_impl_glfw.h
        ${ImGui_DIR}/examples/imgui_impl_glfw.cpp
        ${ImGui_DIR}/examples/imgui_impl_opengl3.h
        ${ImGui_DIR}/examples/imgui_impl_opengl3.cpp
    )

    set(POLLEN3D_OPENGL ON)
    add_compile_definitions(POLLEN3D_OPENGL)

else()
    message("Can't find any ImGui backend")
endif()

