#pragma once

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

#include "viewer3d.h"

#include <iostream>
#include <memory>

#include "p3d/viewer3d/shader_opengl.h"

class Viewer3DOpenGL : public Viewer3D{
public:
    Viewer3DOpenGL() : Viewer3D() {
        m_textureId = new GLuint(0);
        m_shader = std::make_unique<ShaderOpenGL>("../../assets/4.1.shader.vs",
                                                  "../../assets/4.1.shader.fs");
    }

    virtual ~Viewer3DOpenGL() override {
        release();
    }

    void onSizeChanged() override {
        init();
    }

    void release() override {
        glDeleteBuffers(1,&m_Tvbo);
        glDeleteVertexArrays(1,&m_Tvao);
        glDeleteFramebuffers(1,&m_fbo);
        GLuint* tId = (GLuint*)m_textureId;
        if(tId) {
            glDeleteTextures(1,tId);
            *tId = 0;
        }
    }

    void init() override {

        // **** shaders

        glDeleteFramebuffers(1,&m_fbo);
        GLuint* tId = (GLuint*)m_textureId;
        if(tId) {
            glDeleteTextures(1,tId);
            *tId = 0;
        }

        // The texture we're going to render to
        glGenTextures(1, tId);

        // "Bind" the newly created texture : all future texture functions will modify this texture
        glBindTexture(GL_TEXTURE_2D, *tId);

        // Give an empty image to OpenGL ( the last "0" )
        glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, m_width, m_height, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

        // Poor filtering. Needed !
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        glGenFramebuffers(1, &m_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        // Set "renderedTexture" as our colour attachement #0
        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, *tId, 0);

        // Set the list of draw buffers.
        GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cout << "FRAME_BUFFER: " << glCheckFramebufferStatus(GL_FRAMEBUFFER) << std::endl << std::flush;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void drawImpl() override {
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0,0,m_width,m_height);

        glClearColor(m_backColor(0),m_backColor(1),m_backColor(2),m_backColor(3));
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_CLAMP); // BAD !
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);

        if (m_shader){
            m_shader->use();
            m_shader->setVec4("color",1.0f,0.0f,0.5f,1.0f);
            m_shader->setMat4f("camera", m_camera.getWorldToViewMatrix());
            m_shader->setMat4f("world", m_world);
            m_shader->setMat4f("proj", m_camera.getProjMatrix());
//            std::cout << "============================ rand() " << rand() << std::endl;
//            std::cout << m_camera.distance << std::endl;
//            std::cout << m_world << std::endl;
        }

        // ***** draw grid

        if (m_showGrid) {
            const auto & s = m_gridSquareSize;
            if (m_shader) m_shader->setVec4("color",0.33f,0.33f,0.33f,1.0f);
            for (int u = -s/2; u < s/2; ++u) {
                for (int v = -s/2; v < s/2; ++v) {
                    float x = u * s;
                    float y = v * s;

                    glBegin(GL_LINE_LOOP);
                    glVertex3f(x    , y    , 0.0f);
                    glVertex3f(x    , y + s, 0.0f);
                    glVertex3f(x + s, y + s, 0.0f);
                    glVertex3f(x + s, y    , 0.0f);
                    glEnd();
                }
            }
        }

        // ***** draw gizmo

        if (m_showAxis) {
            for (int i = 0; i < 3; i++ ){
                const auto & v = m_axisColors[i];
                if (m_shader) m_shader->setVec4("color",v[0],v[1],v[2],v[3]);
                glBegin(GL_LINES);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(m_gridSquareSize*v[0], m_gridSquareSize*v[1], m_gridSquareSize*v[2]);
                glEnd();
            }
        }

        // ***** draw points

        glBindVertexArray(m_Tvao);
        glPointSize(std::max(1.0f,m_pointSize));
        if (m_shader) m_shader->setVec4("color",1.0f,1.0f,1.0f,1.0f);
        glDrawArrays(GL_POINTS, 0, m_nbPoints);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void setPointCloud(const Mat3X &pcd, Mat3X *colors) override;

private:
    unsigned int m_fbo{0};
    unsigned int m_Tvbo, m_Tvao;

    std::unique_ptr<ShaderOpenGL> m_shader;
};
