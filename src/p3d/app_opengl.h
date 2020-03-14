#pragma once

#include "imgui.h"
#include "examples/imgui_impl_opengl3.h"
#include "examples/imgui_impl_glfw.h"

#include "p3d/app.h"
#include "p3d/viewer3d/viewer3d_opengl.h"

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#include <GLFW/glfw3.h>
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#include <GLFW/glfw3.h>
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
#define GLFW_INCLUDE_NONE         // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>  // Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

class ApplicationOpenGL : public Application
{
public:
    ApplicationOpenGL() {
    }

    ~ApplicationOpenGL() override{
        destroy();
    }

    void destroy() override;

    void init() override;
    void setWindowTitleImpl(std::string str) override;

    bool isRunning() override{ return !glfwWindowShouldClose(m_window);}
    void preLoop() override;
    void postLoop() override;

    virtual void textureBind(const cv::Mat & im) override;
    virtual void textureDisplay(const ImVec2 & size, ImVec2 uv0 = ImVec2(0,0), ImVec2 uv1 = ImVec2(1,1)) override;

private:
    GLFWwindow* m_window = nullptr;
};
