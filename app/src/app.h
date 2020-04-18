#pragma once

#include <future>
#include <string>

#include <opencv2/core.hpp>

#include "imgui.h"
#include "imgui_internal.h"

#include "p3d/project.h"
#include "p3d/data/project_settings.h"
#include "p3d/logger.h"

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
        std::string title = "pollen3d: " + str;
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

    template <typename Func, class... Args>
    void _doHeavyTask(Func f, Args... args)
    {
        m_startedHeavyCalculus = true;
        m_heavyAsyncTask = std::async(std::launch::async, f, args...);
    }

    void _resetAppState()
    {
        setWindowTitle(m_projectData.getProjectPath());
        m_currentSection = Section_Default;
        m_viewer3dNeedsUpdate = true;
        m_textureNeedsUpdate = true;
    }

    enum Tab {
        Tab_General = 0,
        Tab_Image,
        Tab_Stereo,
        Tab_Multiview,
        Tab_PointCloud,
        Tab_COUNT
    };

    template <typename T>
    inline bool isOneOf(const T &v, const std::vector<T> &vec)
    {
        for (size_t i = 0; i < vec.size(); ++i) {
            if (vec[i] == v) return true;
        }
        return false;
    }

    int m_currentTab = -1;
    int m_currentTabForce = -1;
    int m_currentImage = -1;

    enum Section_ {
        Section_Default = 0,
        Section_Matches = Section_Default,
        Section_Epilines,
        Section_Rectified,
        Section_DisparityMap,
    };

    int m_currentSection = Section_Default;

    bool m_showConsole{true};

    // ***** visuals
    int m_width = 1600;
    int m_height = 1000;
    int m_heightTabSection = 70;
    ImGuiTreeNodeFlags m_collapsingHeaderFlags = ImGuiTreeNodeFlags_None;
    bool m_dockingNeedsReset = true;

    ImFont *m_fontMonoSmall = nullptr;
    ImFont *m_fontMono = nullptr;

    void *m_textureId = nullptr;

    int m_textureWidth = 0;
    int m_textureHeight = 0;
    int m_textureScale = 1.0f;
    int m_textureNeedsUpdate = false;
    int m_viewer3dNeedsUpdate = false;
    int m_viewer3dNeedsUpdateVisibility = false;

    std::shared_ptr<WidgetLogger> m_widgetLogger{nullptr};
    std::unique_ptr<Viewer3D> m_viewer3D{nullptr};
    std::unique_ptr<Widget> m_widgetFeat{nullptr};
    std::unique_ptr<Widget> m_widgetMatching{nullptr};
    std::unique_ptr<Widget> m_widgetDenseMatching{nullptr};

    bool m_initialised = false;
    std::string m_execPath = "";

    // *** std::async + wait popup for heavy stuff
    bool m_startedHeavyCalculus = false;

    std::future<void> m_heavyAsyncTask;

    p3d::Project m_projectData;
};
