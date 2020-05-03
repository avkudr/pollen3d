#pragma once

#include <map>

#include "imgui.h"

#include "p3d/core.h"
#include "p3d/logger.h"

#include "eyecamera.h"
#include "pcd_view.h"
#include "camera_view.h"

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

    void setSize(int width, int height);

    void * textureId() { return m_textureId; }

    virtual void init() = 0;
    void draw(int width, int height);

    virtual void drawImpl(int width, int height) = 0;
    virtual void release() = 0;
    virtual void onSizeChanged(){}

    void rotateWorld(float dx, float dy);

    void moveCameraInPlane(float dx, float dy) {
        m_camera.moveUpDown(-dy/(float)m_height);
        m_camera.strafeLeftRight(dx/(float)m_width);
    }

    void moveCameraBackForward(float delta) {
        float coeff = 1.0f - (m_gainZoom * delta);
        m_camera.moveBackForward( coeff );
    }

    p3d::Vec3f m_rotationAxis{p3d::Vec3f(0.0f, 0.0f, 1.0f)};

    virtual void addPointCloud(const std::string& label, const p3d::Mat3Xf& pcd,
                               const p3d::Mat3Xf& colors = {}) = 0;
    void deletePointCloud(const std::string& label) { m_pcd.erase(label); }
    void setPointCloudVisible(const std::string& label, bool visible);
    void deletePointCloudsAll() { m_pcd.clear(); }

    void addCamera(const p3d::Mat34& camera) { LOG_ERR("[void addCamera(const p3d::Mat34& camera)] not implemented"); }
    virtual void addCamera(const p3d::Mat3 &R, const p3d::Vec2 &t) = 0;

    void deleteCamerasAll() { m_cameras.clear(); }


protected:
    void * m_textureId = nullptr;

    int m_width{1024};
    int m_height{1024};

    int m_nbPoints{0};
    p3d::Mat3f m_worldDefaultRotation;
    p3d::Mat4f m_world{p3d::Mat4f::Identity()};

    EyeCamera m_camera;

    // ***** point clouds

    std::map<std::string, std::unique_ptr<PCDView>> m_pcd;
    bool m_pcdTrueColors{true};
    int m_pointSize{2};
    std::array<float, 4> m_pcdColor{0.9f, 0.9f, 0.9f, 1.0f};

    // ***** cameras

    std::vector<std::unique_ptr<CameraView>> m_cameras;

    // ***** Visuals

    bool m_showGrid{true};
    bool m_showAxis{true};
    bool m_showCameras{true};

    float m_gridSquareSize = 200.0f;
    float m_gridNbSquares  = 10;
    std::vector<std::vector<float>> m_axisColors = {
        {1.0f,0.0f,0.0f,1.0f},
        {0.0f,1.0f,0.0f,1.0f},
        {0.0f,0.0f,1.0f,1.0f}
    };
    std::array<float, 4> m_backColor{0.22f, 0.22f, 0.22f, 1.0f};

    // ***** Controls

    float m_gainRotation = 2.0f;
    float m_gainZoom = 1.0f / 25.0f;
};
