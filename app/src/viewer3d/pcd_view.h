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

#include "p3d/core.h"

class ShaderOpenGL;

/** @brief PCDView, point cloud view, is the class responsible for the rendering
 * of one point cloud
 *
 */

class PCDView
{
public:
    PCDView() {}
    virtual ~PCDView() {}

    virtual void init(const p3d::Mat3Xf& pcd,
                      const std::vector<p3d::Vec3uc>& colors = {}) = 0;
    virtual void draw(std::shared_ptr<ShaderOpenGL> shader = nullptr) = 0;

    bool visible() const { return m_visible; }
    void setVisible(bool visible) { m_visible = visible; }

    void setPcdTrueColors(bool pcdTrueColors)
    {
        m_pcdTrueColors = pcdTrueColors;
    }

    void setPointSize(float pointSize) { m_pointSize = pointSize; }

    void setPcdColor(const std::array<float, 4>& pcdColor)
    {
        m_pcdColor = pcdColor;
    }

protected:
    int m_nbPoints{0};
    bool m_visible{true};
    bool m_pcdTrueColors{true};
    float m_pointSize{2.0f};
    std::array<float, 4> m_pcdColor{0.9f, 0.9f, 0.9f, 1.0f};
};
