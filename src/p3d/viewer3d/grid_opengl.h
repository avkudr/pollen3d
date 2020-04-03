#pragma once

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>  // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>  // Initialize with glewInit()
#include <GLFW/glfw3.h>
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
#define GLFW_INCLUDE_NONE  // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/gl/gl.h>
#include <glbinding/glbinding.h>  // Initialize with glbinding::initialize()
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

#include <memory>
#include <vector>

class ShaderOpenGL;

class GridOpenGL
{
public:
    GridOpenGL(const int gridSquareSize) : m_gridSquareSize(gridSquareSize)
    {
        init();
    }
    ~GridOpenGL() {}

    void init();
    void draw(std::shared_ptr<ShaderOpenGL> shader = nullptr);

private:
    GLuint vao{0}, vbo{0}, ibo{0};
    GLuint length;

    int m_gridSquareSize{100};
    std::vector<float> m_color{0.33f, 0.33f, 0.33f, 1.0f};
};
