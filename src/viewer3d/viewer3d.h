#pragma once

#include "imgui.h"

#include "p3d/core.h"

#include "eyecamera.h"

class Viewer3D {
public:
    Viewer3D()
    {
        // some initial world rotation that was determined experimentally :)
        m_worldDefaultRotation << -0.863589f, 0.465148f, 0.0765628f, 0.231463f,
            0.279194f, 0.914598f, 0.410665f, 0.820792f, -0.35449f;

        m_world.block(0, 0, 3, 3) = m_worldDefaultRotation;
    }
    virtual ~Viewer3D(){}

    void setSize(int width, int height) {
        bool sizeChanged = false;
        sizeChanged |= (m_width != width);
        sizeChanged |= (m_height!= height);
        m_width = width;
        m_height = height;

        m_camera.setAspectRatio(width / float(height));

        if (sizeChanged) onSizeChanged();
    }

    void * textureId() { return m_textureId; }

    virtual void init() = 0;
    void draw(int width, int height)
    {
        float wx = ImGui::GetWindowPos().x;
        float wy = ImGui::GetWindowPos().y;

        if (ImGui::IsWindowHovered()) {
            auto io = ImGui::GetIO();

            if (ImGui::IsMouseClicked(2)) {// Left button
                float x = io.MousePos.x - wx;
                float y = io.MousePos.y - wy;
                float w = m_width;
                float h = m_height;
                x = x - w/2.0f;
                y = y - h/2.0f;
                float sphereRadius =
                    w < h ? (w * 0.9f / 2.0f) : (h * 0.9f / 2.0f);

                p3d::Vec3f rotAxis(x, y, 0.0f);
                if (x * x + y * y < sphereRadius * sphereRadius) {
                    rotAxis(2) =
                        sqrtf(sphereRadius * sphereRadius - x * x - y * y);
                }

                rotAxis.normalize();
                m_rotationAxis = rotAxis;
            }

            if (ImGui::IsMouseDragging(2)) {
                if (io.KeyShift)
                    moveCameraInPlane(io.MouseDelta.x, io.MouseDelta.y);
                else
                    rotateWorld(io.MouseDelta.x / m_width, io.MouseDelta.y / m_height);
            }

            if (io.MouseWheel != 0.0f) {

                if (io.KeyAlt) {
                    m_pointSize += 0.25f * io.MouseWheel / abs(io.MouseWheel);
                    m_pointSize = std::max(1.0f,m_pointSize);
                } else {
                    moveCameraBackForward( io.MouseWheel );
                }
            }

            if (ImGui::IsKeyPressed('A'))
                m_showAxis = !m_showAxis;

            if (ImGui::IsKeyPressed('G')) m_showGrid = !m_showGrid;

            if (ImGui::IsKeyPressed('R'))
                m_world.block(0, 0, 3, 3) = m_worldDefaultRotation;
        }

        drawImpl(width, height);
    }

    virtual void drawImpl(int width, int height) = 0;
    virtual void release() = 0;
    virtual void onSizeChanged(){}
    virtual void setPointCloud(const p3d::Mat3Xf& pcd,
                               const p3d::Mat3Xf& colors) = 0;

    void rotateWorld(float dx, float dy) {
        const auto & up = m_camera.getUpAxis();
        p3d::Vec3f right = m_rotationAxis.cross(up);

        if (m_rotationAxis(2) != 0.0f){
            Eigen::AngleAxisf a( m_gainRotation * dx,    up);
            Eigen::AngleAxisf b( m_gainRotation * dy, right);
            m_world.block(0,0,3,3) =
                    b.toRotationMatrix() *
                    a.toRotationMatrix() *
                    m_world.block(0,0,3,3);
        } else {
            const auto& view = m_camera.getViewAxis();
            Eigen::AngleAxisf b( m_gainRotation * dy,view);
            m_world.block(0,0,3,3) =
                    b.toRotationMatrix() *
                    m_world.block(0,0,3,3);
        }
    }

    void moveCameraInPlane(float dx, float dy) {
        m_camera.moveUpDown(-dy/(float)m_height);
        m_camera.strafeLeftRight(dx/(float)m_width);
    }

    void moveCameraBackForward(float delta) {
        float coeff = 1.0f - (m_gainZoom * delta);
        m_camera.moveBackForward( coeff );
    }

    p3d::Vec3f m_rotationAxis{p3d::Vec3f(0.0f, 0.0f, 1.0f)};

protected:
    void * m_textureId = nullptr;

    int m_width{1024};
    int m_height{1024};

    int m_nbPoints{0};
    p3d::Mat3f m_worldDefaultRotation;
    p3d::Mat4f m_world{p3d::Mat4f::Identity()};

    EyeCamera m_camera;

    // ***** Visuals

    bool m_pcdTrueColors{true};
    std::array<float, 4> m_pcdColor{0.9f, 0.9f, 0.9f, 1.0f};
    bool m_showGrid{true};
    bool m_showAxis{true};

    float m_pointSize = 2.5f;
    float m_gridSquareSize = 200.0f;
    float m_gridNbSquares  = 10;
    std::vector<std::vector<float>> m_axisColors = {
        {1.0f,0.0f,0.0f,1.0f},
        {0.0f,1.0f,0.0f,1.0f},
        {0.0f,0.0f,1.0f,1.0f}
    };
    p3d::Vec4f m_backColor{p3d::Vec4f(0.22f, 0.22f, 0.22f, 1.0f)};

    // ***** Controls

    float m_gainRotation = 2.0f;
    float m_gainZoom = 1.0f / 25.0f;
};
