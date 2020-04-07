#include "pcd_view_opengl.h"

#include <vector>

#include "shader_opengl.h"

void PCDViewOpenGL::init(const p3d::Mat3Xf &pcd, const p3d::Mat3Xf &colors)
{
    if (m_Tvbo) glDeleteBuffers(1, &m_Tvbo);
    if (m_Tvao) glDeleteVertexArrays(1, &m_Tvao);

    glDeleteBuffers(1, &m_Tvbo);
    glDeleteVertexArrays(1, &m_Tvao);
    m_nbPoints = 0;

    if (pcd.cols() == 0) return;

    m_nbPoints = pcd.cols();

    Eigen::Matrix<float, 6, Eigen::Dynamic> data;
    data.setOnes(6, m_nbPoints);
    data.topRows(3) = pcd;
    if (colors.cols() == m_nbPoints) data.bottomRows(3) = colors;

    glGenVertexArrays(1, &m_Tvao);
    glGenBuffers(1, &m_Tvbo);
    glBindVertexArray(m_Tvao);
    glBindBuffer(GL_ARRAY_BUFFER, m_Tvbo);
    if (data.Options & Eigen::RowMajor) {
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.size(), data.data(),
                     GL_STATIC_DRAW);
    } else {
        // Create temporary copy of transpose
        auto VT = data.transpose();
        // If its column major then we need to temporarily store a transpose
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * data.size(), VT.data(),
                     GL_STATIC_DRAW);
    }
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PCDViewOpenGL::draw(std::shared_ptr<ShaderOpenGL> shader)
{
    if (m_show && m_nbPoints > 0) {
        glBindVertexArray(m_Tvao);
        glPointSize(std::max(1.0f, m_pointSize));
        if (shader) {
            if (m_pcdTrueColors)
                shader->setVec4("color", 0.0f, 0.0f, 0.0f, 0.0f);
            else
                shader->setVec4("color", m_pcdColor);
        }
        glDrawArrays(GL_POINTS, 0, m_nbPoints);
        glBindVertexArray(0);
    }
}
