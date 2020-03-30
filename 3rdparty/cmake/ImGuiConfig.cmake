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
if (OPENGL_FOUND)
    add_definitions(-DWITH_IMGUI)
    add_definitions(-DWITH_OPENGL)

    message( STATUS "OpenGL - FOUND")
else()
    message( FATAL_ERROR "OpenGL - not found")
endif()

if (WIN32)
    find_package(glfw REQUIRED)
    set(GLAD_DIR ${CMAKE_CURRENT_LIST_DIR}/../glad)

    set(SOURCES
        ${SOURCES}
        ${GLAD_DIR}/include/glad/glad.h
        ${GLAD_DIR}/include/KHR/khrplatform.h
        ${GLAD_DIR}/src/glad.c
        )

    include_directories(
        ${GLAD_DIR}/include/ 
        ${glfw_INCLUDE_DIRS} 
        ${OPENGL_INCLUDE_DIR}
        )
        
    set(ImGui_FOUND true)

    set(ImGui_LIBS ${OPENGL_LIBRARIES} ${glfw_LIBS})
    add_compile_definitions(IMGUI_IMPL_OPENGL_LOADER_GLAD)
else()
    find_package(glfw3 QUIET)
    find_package(GLEW QUIET)
    message(STATUS glfw3_FOUND: ${glfw3_FOUND})
    message(STATUS GLEW_FOUND: ${GLEW_FOUND})

    if(APPLE)
        find_path(GLEW_INCLUDE_DIR GL/glew.h)
    endif()

    if (glfw3_FOUND AND OPENGL_FOUND AND GLEW_FOUND)
        set(ImGui_FOUND true)
        include_directories(
            ${GLFW_INCLUDE_DIR}
            ${GLEW_INCLUDE_DIR}
            ${OPENGL_INCLUDE_DIR}
            )
        set(ImGui_LIBS ${OPENGL_LIBRARIES} glfw GLEW) #glfw3 for mac?

        add_compile_definitions(IMGUI_IMPL_OPENGL_LOADER_GLEW)

    endif()
endif()


if (ImGui_FOUND)
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

