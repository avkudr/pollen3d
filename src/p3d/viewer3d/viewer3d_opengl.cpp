#include "viewer3d_opengl.h"

#include <filesystem>

Viewer3DOpenGL::Viewer3DOpenGL() : Viewer3D()
{
    m_textureId = new GLuint(0);
    auto vertShader = "./assets/4.1.shader.vs";
    auto fragShader = "./assets/4.1.shader.fs";
    m_shader = std::make_unique<ShaderOpenGL>(vertShader, fragShader);
}

void Viewer3DOpenGL::setPointCloud(const Mat3Xf &pcd, const Mat3Xf &colors)
{
    if (pcd.cols() == 0) return;

    m_nbPoints = pcd.cols();

    Eigen::Matrix<float, 6, Eigen::Dynamic> data;
    data.setOnes(6, m_nbPoints);
    data.topRows(3) = pcd;
    if (colors.cols() == m_nbPoints) data.bottomRows(3) = colors;

    glDeleteBuffers(1, &m_Tvbo);
    glDeleteVertexArrays(1, &m_Tvao);

    glGenVertexArrays(1, &m_Tvao);
    glGenBuffers(1, &m_Tvbo);
    glBindVertexArray(m_Tvao);
    glBindBuffer(GL_ARRAY_BUFFER, m_Tvbo);
    if (data.Options & Eigen::RowMajor) {
        glBufferData(
            GL_ARRAY_BUFFER,
            sizeof(float) * data.size(),
            data.data(),
            GL_STATIC_DRAW);
    } else {
        // Create temporary copy of transpose
        auto VT = data.transpose();
        // If its column major then we need to temporarily store a transpose
        glBufferData(
            GL_ARRAY_BUFFER,
            sizeof(float) * data.size(),
            VT.data(),
            GL_STATIC_DRAW);
    }
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
