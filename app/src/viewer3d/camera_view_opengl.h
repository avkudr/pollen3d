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

#include "camera_view.h"

class ShaderOpenGL;

/** @brief CameraView, point cloud view, is the class responsible for the rendering
 * of one point cloud
 *
 */

class CameraViewOpenGL : public CameraView
{
public:
    CameraViewOpenGL() {}
    virtual ~CameraViewOpenGL() {}

    virtual void init(const p3d::Mat3& R, const p3d::Vec2& t) override;
    virtual void draw(std::shared_ptr<ShaderOpenGL> shader = nullptr, int idx = 0) override;

protected:
    GLuint vao{0};
    std::vector<GLuint> indices;  // Vec4uint
};
