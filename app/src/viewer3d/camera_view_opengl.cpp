#include "camera_view_opengl.h"

#include "p3d/core.h"
#include "shader_opengl.h"

void CameraViewOpenGL::init(const p3d::Mat3 &R, const p3d::Vec2 &t) {
    std::vector<p3d::Vec3f> vertices;  // Vec3f

    const auto aspect = 3.0f/4.0f;
    const auto width = 40.0f;

    /*
     *    0 --- 1
     *    |     |
     *    |  4  |
     *    |     |
     *    3 --- 2
     */

    vertices.emplace_back(p3d::Vec3f(-width, -width*aspect,  0.0f));
    vertices.emplace_back(p3d::Vec3f( width, -width*aspect,  0.0f));
    vertices.emplace_back(p3d::Vec3f( width,  width*aspect,  0.0f));
    vertices.emplace_back(p3d::Vec3f(-width,  width*aspect,  0.0f));
    vertices.emplace_back(p3d::Vec3f(     0,             0, 2*width));

    p3d::Mat3f Rf = R.cast<float>();
    p3d::Vec3f tf;
    tf << 0,0,-1000.0;

    for (auto & v : vertices) {
        v = Rf.transpose() * v - Rf.transpose() * tf;
    }

    indices = std::vector<GLuint>({0,1,4,1,4,2,2,4,3,3,4,0});
    //indices.emplace_back(std::array<GLuint, 3>({0,3,2}));

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float) * 3,
                 vertices[0].data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void CameraViewOpenGL::draw(std::shared_ptr<ShaderOpenGL> shader, int idx)
{
    if (shader) {
        if (idx == 0) shader->setVec4("color", 1.0f, 0.0f, 0.0f, 1.0f);
        else shader->setVec4("color", 0.8f, 0.8f, 0.8f, 1.0f);
    }

    glBindVertexArray(vao);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, indices.size() ,GL_UNSIGNED_INT,indices.data());
    glBindVertexArray(0);
}
