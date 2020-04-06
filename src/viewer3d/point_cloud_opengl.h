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

#include "point_cloud.h"

class ShaderOpenGL;

class PointCloudOpenGL : public PointCloud
{
public:
    PointCloudOpenGL() = delete;
    PointCloudOpenGL(const p3d::Mat3Xf& pcd, const p3d::Mat3Xf& colors = {})
        : PointCloud()
    {
        init(pcd, colors);
    }
    virtual ~PointCloudOpenGL()
    {
        if (m_Tvbo) glDeleteBuffers(1, &m_Tvbo);
        if (m_Tvao) glDeleteVertexArrays(1, &m_Tvao);
    }

    void init(const p3d::Mat3Xf& pcd, const p3d::Mat3Xf& colors = {}) override;
    void draw(std::shared_ptr<ShaderOpenGL> shader = nullptr) override;

private:
    unsigned int m_Tvbo{0}, m_Tvao{0};
};
