#include "grid_opengl.h"

#include <vector>

#include "shader_opengl.h"

void GridOpenGL::init()
{
    std::vector<std::array<float, 3>> vertices;  // Vec3f
    std::vector<std::array<GLuint, 4>> indices;  // Vec4uint

    const auto& s = m_gridSquareSize;
    GLuint i = 0;
    for (int u = -s / 2; u < s / 2; ++u) {
        for (int v = -s / 2; v < s / 2; ++v) {
            float x = u * s;
            float y = v * s;

            vertices.emplace_back(std::array<float, 3>({x, y, 0.0f}));
            vertices.emplace_back(std::array<float, 3>({x, y + s, 0.0f}));
            vertices.emplace_back(std::array<float, 3>({x + s, y + s, 0.0f}));
            vertices.emplace_back(std::array<float, 3>({x + s, y, 0.0f}));

            indices.emplace_back(std::array<GLuint, 4>(
                {4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3}));
            i++;
        }
    }

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float) * 3,
                 vertices[0].data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    GLuint ibo;
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint) * 4,
                 indices[0].data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    length = (GLuint)indices.size() * 4;
}

void GridOpenGL::draw(std::shared_ptr<ShaderOpenGL> shader)
{
    if (shader) {
        shader->setVec4("color", m_color[0], m_color[1], m_color[2],
                        m_color[3]);
    }

    glBindVertexArray(vao);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawArrays(GL_QUADS, 0, length);
    glBindVertexArray(0);
}
