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

#include "p3d/viewer3d/shader_opengl.h"

class Grid
{
public:
    Grid() { init(); }
    void init()
    {
        int slices = 100;
        std::vector<Vec3f> vertices;
        std::vector<Eigen::Matrix<GLuint, 3, 1>> indices;

        for (int j = 0; j <= slices; ++j) {
            for (int i = 0; i <= slices; ++i) {
                float x = (float)i / (float)slices;
                float y = 0;
                float z = (float)j / (float)slices;
                vertices.emplace_back(x, y, z);
            }
        }

        for (int j = 0; j < slices; ++j) {
            for (int i = 0; i < slices; ++i) {
                int row1 = j * (slices + 1);
                int row2 = (j + 1) * (slices + 1);

                indices.emplace_back(row1 + i, row1 + i + 1, row2 + i + 1);
                indices.emplace_back(row1 + i, row2 + i + 1, row2 + i);
            }
        }

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        GLuint vbo;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float) * 3, vertices[0].data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        GLuint ibo;
        glGenBuffers(1, &ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint) * 3, indices[0].data(), GL_STATIC_DRAW);

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        lenght = (GLuint)indices.size() * 3;
    }
    void draw()
    {
        std::cout << "drawing grid " << rand() << std::endl;
        glBindVertexArray(vao);
        glDrawElements(GL_LINES, lenght, GL_UNSIGNED_INT, NULL);
        glBindVertexArray(0);
    }

private:
    GLuint vao{0}, vbo{0}, ibo{0};
    GLuint lenght;
};

class Viewer3DOpenGL : public Viewer3D
{
public:
    Viewer3DOpenGL();

    virtual ~Viewer3DOpenGL() override
    {
        release();
    }

    void onSizeChanged() override
    {
        init();
    }

    void release() override
    {
        if (m_Tvao) glDeleteVertexArrays(1, &m_Tvao);
        if (m_Tvbo) glDeleteBuffers(1, &m_Tvbo);
        GLuint* tId = (GLuint*)m_textureId;
        if (tId) delete (GLuint*)m_textureId;
        if (m_fbo) glDeleteFramebuffers(1, &m_fbo);
    }

    void init() override
    {
        // **** shaders

        glDeleteFramebuffers(1, &m_fbo);
        GLuint* tId = (GLuint*)m_textureId;
        if (tId) {
            glDeleteTextures(1, tId);
            *tId = 0;
        }

        // The texture we're going to render to
        glGenTextures(1, tId);

        // "Bind" the newly created texture : all future texture functions will modify this texture
        glBindTexture(GL_TEXTURE_2D, *tId);

        // Give an empty image to OpenGL ( the last "0" )
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

        // Poor filtering. Needed !
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        glGenFramebuffers(1, &m_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        // Set "renderedTexture" as our colour attachement #0
        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, *tId, 0);

        // Set the list of draw buffers.
        GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1, DrawBuffers);  // "1" is the size of DrawBuffers

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cout << "FRAME_BUFFER: " << glCheckFramebufferStatus(GL_FRAMEBUFFER) << std::endl
                      << std::flush;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void drawImpl() override
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0, 0, m_width, m_height);

        glClearColor(m_backColor(0), m_backColor(1), m_backColor(2), m_backColor(3));
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_CLAMP);  // BAD !
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);

        if (m_shader) {
            m_shader->use();
            m_shader->setVec4("color", 1.0f, 0.0f, 0.5f, 1.0f);
            m_shader->setMat4f("camera", m_camera.getWorldToViewMatrix());
            m_shader->setMat4f("world", m_world);
            m_shader->setMat4f("proj", m_camera.getProjMatrix());
            //            std::cout << "============================ rand() " << rand() << std::endl;
            //            std::cout << m_camera.distance << std::endl;
            //            std::cout << m_world << std::endl;
        }

        // ***** draw grid

        if (m_showGrid) {
            grid.draw();
            const auto& s = m_gridSquareSize;
            if (m_shader) m_shader->setVec4("color", 0.33f, 0.33f, 0.33f, 1.0f);
            for (int u = -s / 2; u < s / 2; ++u) {
                for (int v = -s / 2; v < s / 2; ++v) {
                    float x = u * s;
                    float y = v * s;

                    // glBegin(GL_LINE_LOOP);
                    // glVertex3f(x    , y    , 0.0f);
                    // glVertex3f(x    , y + s, 0.0f);
                    // glVertex3f(x + s, y + s, 0.0f);
                    // glVertex3f(x + s, y    , 0.0f);
                    // glEnd();
                }
            }
        }

        // ***** draw gizmo

        if (m_showAxis) {
            for (int i = 0; i < 3; i++) {
                const auto& v = m_axisColors[i];
                if (m_shader) m_shader->setVec4("color", v[0], v[1], v[2], v[3]);
                // glBegin(GL_LINES);
                // glVertex3f(0.0f, 0.0f, 0.0f);
                // glVertex3f(m_gridSquareSize*v[0], m_gridSquareSize*v[1], m_gridSquareSize*v[2]);
                // glEnd();
            }
        }

        // ***** draw points

        glBindVertexArray(m_Tvao);
        glPointSize(std::max(1.0f, m_pointSize));
        if (m_shader) m_shader->setVec4("color", 0.0f, 0.0f, 0.0f, 0.0f);
        glDrawArrays(GL_POINTS, 0, m_nbPoints);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void setPointCloud(const Mat3Xf& pcd, const Mat3Xf& colors) override;

private:
    unsigned int m_fbo{0};
    unsigned int m_Tvbo{0}, m_Tvao{0};

    std::unique_ptr<ShaderOpenGL> m_shader{nullptr};

    Grid grid;
};
