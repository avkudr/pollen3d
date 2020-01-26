#pragma once

#include <future>
#include <string>

#include "imgui.h"
#include "imgui_internal.h"

#include "opencv2/core.hpp"

#include "p3d/console_logger.h"
#include "p3d/data/project_data.h"
#include "p3d/data/project_settings.h"


class Application
{
public:
    Application() {
        LOG_INFO("Welcome to Pollen3D");
    }

    virtual ~Application() { destroy(); }

    virtual void destroy() {}

    virtual void init() = 0;
    void run(){
        while(isRunning()){
            preLoop();
            draw(m_width,m_height);
            postLoop();
        }
    }

    virtual bool isRunning() = 0;
    virtual void preLoop() = 0;
    virtual void postLoop() = 0;

    bool isTextureReady() const { return m_textureWidth > 0 && m_textureHeight > 0; }
    virtual void textureBind(const cv::Mat & im) = 0;
    virtual void textureDisplay(float w, float h) = 0;

    void initImGui();
    void applyStyle();
    void draw(int width, int height);

protected:

    void _drawMenuBar();
    void _drawTab();
    void _drawTab_General();
    void _drawTab_Image();
    void _drawTab_Stereo();
    void _drawData();
    void _drawProperties();
    void _drawCentral();
    void _processKeyboardInput();

    void _showFeatures(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float featuresSize = 2.0f);
    void _showMatches(const ImVec2 & pos, const ImVec2 & size, const ImVec4 & col, float lineWidth = 1.0f);

    template<typename Scalar, int SizeX, int SizeY>
    void drawProperty_matrix(const Eigen::Matrix<Scalar, SizeX, SizeY> &v, const std::string &name);

    template<typename Type>
    void drawProperty_basic(const Type &v, const std::string &name, const char * fmt);

    template<typename Func>
    void _doHeavyTask(Func f){
        m_startedHeavyCalculus = true;
        m_heavyAsyncTask = std::async(std::launch::async,f);
    }

    enum Tab{
        Tab_General = 0,
        Tab_Image      ,
        Tab_Stereo     ,
        Tab_Multiview  ,
        Tab_PointCloud ,
        Tab_COUNT
    };

    template<typename T>
    inline bool isOneOf(const T & v, const std::vector<T> vec) {
        for (size_t i = 0; i < vec.size(); ++i) {
            if (vec[i] == v) return true;
        }
        return false;
    }

    int m_currentTab = -1;
    int m_currentImage = -1;

    int m_width = 1600;
    int m_height = 1000;
    int m_heightTabSection = 200;
    bool m_dockingNeedsReset = true;

    ImFont * m_fontMonoSmall = nullptr;
    int m_textureWidth  = 0;
    int m_textureHeight = 0;
    int m_textureScale = 1.0f;
    int m_textureNeedsUpdate = false;

    bool m_initialised = false;

    // *** std::async + wait popup for heavy stuff
    bool m_startedHeavyCalculus= false;
    std::future<void> m_heavyAsyncTask;

    ProjectData m_projectData;
};
