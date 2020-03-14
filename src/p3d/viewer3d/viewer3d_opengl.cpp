#include "viewer3d_opengl.h"

void Viewer3DOpenGL::setPointCloud(const Mat3X &pcd, Mat3X *colors)
{
    if (pcd.cols() == 0) return;

    Eigen::Matrix<float,3,Eigen::Dynamic> V = pcd.cast<float>();

    m_nbPoints = V.cols();

    glDeleteBuffers(1,&m_Tvbo);
    glDeleteVertexArrays(1,&m_Tvao);

    glGenVertexArrays(1, &m_Tvao);
    glGenBuffers(1, &m_Tvbo);
    glBindVertexArray(m_Tvao);
    glBindBuffer(GL_ARRAY_BUFFER, m_Tvbo);
    if(V.Options & Eigen::RowMajor)
    {
      glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(float)*V.size(),
        V.data(),
        GL_STATIC_DRAW);
    }else
    {
      // Create temporary copy of transpose
      auto VT = V.transpose();
      // If its column major then we need to temporarily store a transpose
      glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(float)*V.size(),
        VT.data(),
        GL_STATIC_DRAW);
    }
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
