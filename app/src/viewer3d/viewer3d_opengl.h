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

#include "viewer3d.h"

#include <iostream>
#include <memory>

#include "grid_opengl.h"
#include "shader_opengl.h"

class Viewer3DOpenGL : public Viewer3D
{
public:
    Viewer3DOpenGL(const char* version = NULL);

    virtual ~Viewer3DOpenGL() override { release(); }
    void onSizeChanged() override { init(); }

    void release() override;
    void init() override;
    void drawImpl(int width, int height) override;
    void addPointCloud(const std::string& label, const p3d::Mat3Xf& pcd,
                       const p3d::Mat3Xf& colors = {}) override;

    void addCamera(const p3d::Mat3 &R, const p3d::Vec2 &t) override;

private:
    unsigned int m_fbo{0};
    unsigned int m_Tvbo{0}, m_Tvao{0};

    std::shared_ptr<ShaderOpenGL> m_shader{nullptr};
    std::unique_ptr<GridOpenGL> m_grid{nullptr};
};
