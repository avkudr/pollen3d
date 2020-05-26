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

#include "pcd_view.h"

class ShaderOpenGL;

class PCDViewOpenGL : public PCDView
{
public:
    PCDViewOpenGL() = delete;
    PCDViewOpenGL(const p3d::Mat3Xf& pcd, const std::vector<p3d::Vec3uc>& colors = {})
        : PCDView()
    {
        init(pcd, colors);
    }
    virtual ~PCDViewOpenGL()
    {
        if (m_Tvbo) glDeleteBuffers(1, &m_Tvbo);
        if (m_Tvao) glDeleteVertexArrays(1, &m_Tvao);
    }

    void init(const p3d::Mat3Xf& pcd,
              const std::vector<p3d::Vec3uc>& colors = {}) override;
    void draw(std::shared_ptr<ShaderOpenGL> shader = nullptr) override;

private:
    unsigned int m_Tvbo{0}, m_Tvao{0};
};
