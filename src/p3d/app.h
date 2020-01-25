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
            update(m_width,m_height);
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
    void update(int width, int height);

protected:

    void _renderMenuBar();
    void _renderTabWidget();
    void _renderTab_General();
    void _renderTab_Image();
    void _renderTab_Stereo();
    void _renderLeftWidget();
    void _renderCentralWidget();
    void _processKeyboardInput();

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
    int m_heightTabSection = 250;

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
