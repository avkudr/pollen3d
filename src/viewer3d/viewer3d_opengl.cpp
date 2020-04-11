#include "viewer3d_opengl.h"

#include <filesystem>

#include "p3d/logger.h"

#include "pcd_view_opengl.h"

using namespace p3d;

Viewer3DOpenGL::Viewer3DOpenGL() : Viewer3D()
{
    m_textureId = new GLuint(0);

#ifdef __APPLE__
    std::string version = "410";
#else
    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    std::string version = std::to_string(major * 100 + minor * 10);
#endif

    LOG_INFO("glsl version: %s", version.c_str());

    std::string vertexShader =
        "#version " + version +
        "\n"
        "layout (location = 0) in vec3 vertexPosition;\n"
        "layout (location = 1) in vec3 vertexColor;\n"
        "uniform vec4 color;\n"
        "uniform mat4 camera;\n"
        "uniform mat4 world;\n"
        "uniform mat4 proj;\n"
        "out vec4 ourColor;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    if (color.w == 0.0) {\n"
        "        ourColor = vec4(vertexColor, 1.0);\n"
        "    } else {\n"
        "        ourColor = color;\n"
        "    }\n"
        "    gl_Position = proj * camera * world * "
        "                  "
        "vec4(vertexPosition.x,vertexPosition.y,-vertexPosition.z,1.0);\n"
        "}\n";

    std::string fragmentShader = "#version " + version +
                                 "\n"
                                 "out vec4 FragColor;\n        "
                                 "in vec4 ourColor;\n          "
                                 " \n                          "
                                 "void main()\n                "
                                 "{\n                          "
                                 "    FragColor = ourColor;\n  "
                                 "}\n";

    m_shader = std::make_shared<ShaderOpenGL>(vertexShader, fragmentShader);
    m_grid = std::make_unique<GridOpenGL>(100);
}

void Viewer3DOpenGL::release()
{
    if (m_Tvao) glDeleteVertexArrays(1, &m_Tvao);
    if (m_Tvbo) glDeleteBuffers(1, &m_Tvbo);
    GLuint *tId = (GLuint *)m_textureId;
    if (tId) delete (GLuint *)m_textureId;
    if (m_fbo) glDeleteFramebuffers(1, &m_fbo);
}

void Viewer3DOpenGL::init()
{
    // **** shaders

    glDeleteFramebuffers(1, &m_fbo);
    GLuint *tId = (GLuint *)m_textureId;
    if (tId) {
        glDeleteTextures(1, tId);
        *tId = 0;
    }

    // The texture we're going to render to
    glGenTextures(1, tId);

    // "Bind" the newly created texture : all future texture functions will
    // modify this texture
    glBindTexture(GL_TEXTURE_2D, *tId);

    // Give an empty image to OpenGL ( the last "0" )
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_width, m_height, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, 0);

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
        std::cout << "FRAME_BUFFER: "
                  << glCheckFramebufferStatus(GL_FRAMEBUFFER) << std::endl
                  << std::flush;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Viewer3DOpenGL::drawImpl(int width, int height)
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_width, m_height);

    glClearColor(m_backColor(0), m_backColor(1), m_backColor(2),
                 m_backColor(3));
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
    }

    // ***** draw grid

    if (m_showGrid && m_grid) { m_grid->draw(m_shader); }

    // ***** draw gizmo

    if (m_showAxis) {
        for (int i = 0; i < 3; i++) {
            const auto &v = m_axisColors[i];
            if (m_shader) m_shader->setVec4("color", v[0], v[1], v[2], v[3]);
            // glBegin(GL_LINES);
            // glVertex3f(0.0f, 0.0f, 0.0f);
            // glVertex3f(m_gridSquareSize*v[0], m_gridSquareSize*v[1],
            // m_gridSquareSize*v[2]); glEnd();
        }
    }

    // ***** draw points

    for (const auto &pcd : m_pcd) {
        if (!pcd.second) continue;
        if (pcd.second->visible()) {
            pcd.second->setPcdTrueColors(m_pcdTrueColors);
            pcd.second->setPointSize(m_pointSize);
            pcd.second->setPcdColor(m_pcdColor);
            pcd.second->draw(m_shader);
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    GLuint *tId = (GLuint *)m_textureId;
    if (m_textureId) {
        ImGui::Image(reinterpret_cast<ImTextureID>(*tId), ImVec2(width, height),
                     ImVec2(0, 0), ImVec2(1, 1));
    }
    ImGui::SetCursorPos(ImVec2(15, 35));
    ImGui::BeginGroup();
    ImGui::Text("<G> to set the grid %s", m_showGrid ? "off" : "on");
    ImGui::Text("<R> to reset view");
    ImGui::Text("<Alt + Wheel> to change point size");

    ImGui::Checkbox("true colors", &m_pcdTrueColors);
    if (!m_pcdTrueColors) {
        ImGui::SameLine();
        ImGui::ColorEdit4(
            "point cloud color##pcd_color", m_pcdColor.data(),
            ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
    }
    ImGui::EndGroup();

    //    painter.drawText(10,15,"Press M to on/off the mesh");

    //    painter.drawText(10,30,"Press Ctrl + Wheel to change the focal
    //    length");

    //    painter.drawText(10,45,"Press  Alt + Wheel to size of points");

    //    painter.drawText(10, height()-70, "Camera distance : " +
    //    QString::number(cam.getDistance()));

    //    painter.drawText(10, height()-55, "Camera wide angle : " +
    //    QString::number(cam.getVertAngle()));

    //    painter.drawText(10, height()-40, "Vertices number : " +
    //    QString::number(m_vertArr.size()));

    //    painter.drawText(10, height()-25, "Faces number : " +
    //    QString::number(m_indexArr.size() / 3));
}

void Viewer3DOpenGL::addPointCloud(const std::string &label, const Mat3Xf &pcd,
                                   const Mat3Xf &colors)
{
    if (pcd.cols() == 0) return;
    m_pcd.insert({label, std::make_unique<PCDViewOpenGL>(pcd, colors)});
}
