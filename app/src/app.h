#pragma once

#include <future>
#include <string>

#include <opencv2/core.hpp>

#include "imgui.h"
#include "imgui_internal.h"

#include "p3d/project.h"
#include "p3d/data/project_settings.h"
#include "p3d/logger.h"

#include "common/app_state.h"
#include "version.h"
#include "viewer3d/viewer3d.h"
#include "widgets/widget.h"
#include "widgets/widget_console.h"

class Application
{
public:
    Application();

    virtual ~Application() { destroy(); }

    virtual void destroy() {}

    virtual void init() = 0;

    void setWindowTitle(std::string str)
    {
        if (str == "") str = "*";
        std::string title = "pollen3d v" + std::to_string(POLLEN3D_VERSION_MAJOR) + "." +
                            std::to_string(POLLEN3D_VERSION_MINOR) + "." +
                            std::to_string(POLLEN3D_VERSION_PATCH) + ": " + str;
        setWindowTitleImpl(title);
    }

    void run()
    {
        while (isRunning()) {
            preLoop();
            draw(m_width, m_height);
            postLoop();
        }
        m_viewer3D.reset();
    }

    virtual bool isRunning() = 0;
    virtual void preLoop() = 0;
    virtual void postLoop() = 0;

    bool isTextureReady() const { return m_textureId && m_textureWidth > 0 && m_textureHeight > 0; }
    virtual void textureBind(const cv::Mat &im) = 0;
    virtual void textureDisplay(const ImVec2 &size, ImVec2 uv0 = ImVec2(0, 0), ImVec2 uv1 = ImVec2(1, 1)) = 0;

    void initImGui();
    void applyStyle();
    void draw(int width, int height);

    void setTextureId(void *textureId) { m_textureId = textureId; }

    void openProject(const std::string &projPath);
    void saveProject();

protected:
    virtual void setWindowTitleImpl(std::string str) = 0;

    void _drawMenuBar(int width);
    void _drawControls();
    void _drawTab_Image();
    void _drawTab_Stereo();
    void _drawTab_Multiview();
    void _drawTab_PointCloud();
    void _drawData();
    void _drawProperties();
    void _drawCentral();
    void _processKeyboardInput();

    void _showFeatures(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float featuresSize = 2.0f);
    void _showMatches(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float lineWidth = 1.0f, int skipEvery = 1);
    void _showEpilines(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float lineWidth = 1.0f, int skipEvery = 1);

    template <typename Scalar, int SizeX, int SizeY>
    void drawProperty_matrix(const Eigen::Matrix<Scalar, SizeX, SizeY> &v, const std::string &name,
                             const std::string &longName = "");

    template <typename Type>
    void drawProperty_basic(const Type &v, const std::string &name, const char *fmt, const char *icon = nullptr);

    void _resetAppState()
    {
        setWindowTitle(m_projectData.getProjectPath());
        m_state.reset();
        m_state.setSection(Section_Default);
    }

    template <typename T>
    inline bool isOneOf(const T &v, const std::vector<T> &vec)
    {
        for (size_t i = 0; i < vec.size(); ++i) {
            if (vec[i] == v) return true;
        }
        return false;
    }

    int m_currentTabForce = -1;

    bool m_showConsole{true};

    // ***** visuals
    int m_width = 1600;
    int m_height = 1000;
    int m_heightTabSection = 70;
    bool m_dockingNeedsReset = true;

    ImFont *m_fontMonoSmall = nullptr;
    ImFont *m_fontMono = nullptr;

    void *m_textureId = nullptr;

    int m_textureWidth = 0;
    int m_textureHeight = 0;
    int m_textureScale = 1.0f;

    std::shared_ptr<WidgetLogger> m_widgetLogger{nullptr};
    std::unique_ptr<Viewer3D> m_viewer3D{nullptr};
    std::unique_ptr<Widget> m_widgetFeat{nullptr};
    std::unique_ptr<Widget> m_widgetMatching{nullptr};
    std::unique_ptr<Widget> m_widgetDenseMatching{nullptr};

    p3d::Project m_projectData;
    AppState m_state;
};
