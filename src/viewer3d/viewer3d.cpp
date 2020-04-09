#include "viewer3d.h"

void Viewer3D::setSize(int width, int height)
{
    bool sizeChanged = false;
    sizeChanged |= (m_width != width);
    sizeChanged |= (m_height != height);
    m_width = width;
    m_height = height;

    m_camera.setAspectRatio(width / float(height));

    if (sizeChanged) onSizeChanged();
}

void Viewer3D::draw(int width, int height)
{
    float wx = ImGui::GetWindowPos().x;
    float wy = ImGui::GetWindowPos().y;

    if (ImGui::IsWindowHovered()) {
        auto io = ImGui::GetIO();

        if (ImGui::IsMouseClicked(2)) {  // Left button
            float x = io.MousePos.x - wx;
            float y = io.MousePos.y - wy;
            float w = m_width;
            float h = m_height;
            x = x - w / 2.0f;
            y = y - h / 2.0f;
            float sphereRadius = w < h ? (w * 0.9f / 2.0f) : (h * 0.9f / 2.0f);

            p3d::Vec3f rotAxis(x, y, 0.0f);
            if (x * x + y * y < sphereRadius * sphereRadius) {
                rotAxis(2) = sqrtf(sphereRadius * sphereRadius - x * x - y * y);
            }

            rotAxis.normalize();
            m_rotationAxis = rotAxis;
        }

        if (ImGui::IsMouseDragging(2)) {
            if (io.KeyShift)
                moveCameraInPlane(io.MouseDelta.x, io.MouseDelta.y);
            else
                rotateWorld(io.MouseDelta.x / m_width,
                            io.MouseDelta.y / m_height);
        }

        if (io.MouseWheel != 0.0f) {
            if (io.KeyAlt) {
                m_pointSize += 0.25f * io.MouseWheel / abs(io.MouseWheel);
                m_pointSize = std::max(1.0f, m_pointSize);
            } else {
                moveCameraBackForward(io.MouseWheel);
            }
        }

        if (ImGui::IsKeyPressed('A')) m_showAxis = !m_showAxis;

        if (ImGui::IsKeyPressed('G')) m_showGrid = !m_showGrid;

        if (ImGui::IsKeyPressed('R')) {
            m_world.block(0, 0, 3, 3) = m_worldDefaultRotation;
            m_camera.resetParams();
        }
    }

    drawImpl(width, height);
}

void Viewer3D::rotateWorld(float dx, float dy)
{
    const auto& up = m_camera.getUpAxis();
    p3d::Vec3f right = m_rotationAxis.cross(up);

    if (m_rotationAxis(2) != 0.0f) {
        Eigen::AngleAxisf a(m_gainRotation * dx, up);
        Eigen::AngleAxisf b(m_gainRotation * dy, right);
        m_world.block(0, 0, 3, 3) = b.toRotationMatrix() *
                                    a.toRotationMatrix() *
                                    m_world.block(0, 0, 3, 3);
    } else {
        const auto& view = m_camera.getViewAxis();
        Eigen::AngleAxisf b(m_gainRotation * dy, view);
        m_world.block(0, 0, 3, 3) =
            b.toRotationMatrix() * m_world.block(0, 0, 3, 3);
    }
}

void Viewer3D::setPointCloudVisible(const std::string& label, bool visible)
{
    if (m_pcd.count(label) == 0) return;
    m_pcd[label]->setVisible(visible);
}
