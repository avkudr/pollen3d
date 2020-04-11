#include "app.h"

#include <filesystem>
#include <future>

#include "../assets/fonts/IconsFontAwesome5.h"

#include "p3d/commands.h"
#include "p3d/project_manager.h"

#include "gui/common.h"
#include "gui/imgui_custom.h"
#include "gui/palette.h"
#include "gui/plots.h"

#include "gui/widget_dense_matching.h"
#include "gui/widget_feature_extract.h"
#include "gui/widget_matching.h"

#define COLOR_GREEN \
    ImVec4 { 0.0f, 0.7f, 0.3f, 1.0f }
#define COLOR_PINK \
    ImVec4 { 1.0f, 0.3f, 0.6f, 1.0f }

using namespace p3d;

Application::Application()
{
    m_widgetLogger = std::make_shared<WidgetLogger>();
    p3d::_logger = m_widgetLogger;

    m_widgetFeat = std::make_unique<WidgetFeatureExtract>();
    m_widgetMatching = std::make_unique<WidgetMatching>();
    m_widgetDenseMatching = std::make_unique<WidgetDenseMatching>();
    LOG_OK("Welcome to Pollen3D!");
}

void Application::initImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;

    std::string font = "SourceSansPro-Regular.ttf";
    std::string fontMono = "UbuntuMono-R.ttf";

    std::string pathToFonts = "./assets/fonts/";
    io.Fonts->AddFontFromFileTTF(std::string(pathToFonts + font).c_str(),
                                 18.0f);

    static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
    ImFontConfig icons_config;
    icons_config.MergeMode = true;
    icons_config.PixelSnapH = true;
    io.Fonts->AddFontFromFileTTF(
        std::string(pathToFonts + FONT_ICON_FILE_NAME_FAS).c_str(), 16.0f,
        &icons_config, icons_ranges);

    m_fontMono = io.Fonts->AddFontFromFileTTF(
        std::string(pathToFonts + fontMono).c_str(), 16.0f);
    m_widgetLogger->setFont(m_fontMono);
    m_fontMonoSmall = io.Fonts->AddFontFromFileTTF(
        std::string(pathToFonts + fontMono).c_str(), 14.0f);

    applyStyle();
}

void Application::applyStyle()
{
    ImGui::StyleColorsDark();

    ImGuiStyle &style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.FrameRounding = 4;
    style.FramePadding = ImVec2(4, 2);
    style.WindowRounding = 0;
    style.WindowBorderSize = 1;
    style.IndentSpacing = 8.0f;
    style.WindowMenuButtonPosition = ImGuiDir_Right;
    style.Colors[ImGuiCol_TitleBg] = style.Colors[ImGuiCol_TitleBgActive];

    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.11f, 0.11f, 1.0f);
    style.Colors[ImGuiCol_ChildBg] = ImVec4(0.145f, 0.145f, 0.145f, 1.0f);
    style.Colors[ImGuiCol_TitleBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.0f);
    style.Colors[ImGuiCol_TitleBgActive] = style.Colors[ImGuiCol_TitleBg];
    style.Colors[ImGuiCol_TabUnfocusedActive] = style.Colors[ImGuiCol_TitleBg];
    style.Colors[ImGuiCol_Header] = ImVec4(0.216f, 0.216f, 0.239f, 1.0f);
    style.Colors[ImGuiCol_Tab] = ImVec4(0.33f, 0.33f, 0.33f, 0.86f);
}

void Application::draw(int width, int height)
{
    ImGui::NewFrame();

    _drawMenuBar(width);

    ImVec4 colorBg2 = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    colorBg2.w = 1.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0);

    ImGui::SetNextWindowPos(ImVec2(0, m_heightTabSection));
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(width, height - m_heightTabSection),
        ImVec2(width, height - m_heightTabSection));
    ImGuiID dock_id = ImGui::GetID("ID().c_str()");
    if (ImGui::Begin("CentralArea", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                         ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        ImGui::DockSpace(dock_id, ImVec2(0, 0),
                         ImGuiDockNodeFlags_NoCloseButton |
                             ImGuiDockNodeFlags_NoWindowMenuButton);
        ImGui::End();
    }

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus;

    if (ImGui::Begin("Controls", nullptr, flags)) {
        _drawControls();
        ImGui::End();
    }

    ImGui::Begin("Data", nullptr, flags);
    _drawData();
    ImGui::End();

    ImGui::Begin("Properties", nullptr, flags);
    _drawProperties();
    ImGui::End();

    ImGui::PushStyleColor(ImGuiCol_ChildBg, colorBg2);
    ImGui::Begin("Viewer", nullptr,
                 flags | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse);
    _drawCentral();
    ImGui::End();
    ImGui::PopStyleColor();

    if (m_showConsole && m_widgetLogger) {
        ImGui::Begin("Console", nullptr, flags);
        m_widgetLogger->render();
        ImGui::End();
    }

    ImGui::PopStyleVar();

    if (m_dockingNeedsReset) {
        m_showConsole = true;
        const int preferredControlsWidth = 450;
        const float preferredConsoleFrac = 0.35f;

        ImGuiID dock_id = ImGui::GetID("ID().c_str()");

        ImGui::DockBuilderRemoveNode(dock_id);  // Clear out existing layout
        ImGui::DockBuilderAddNode(dock_id,      // Add empty node
                                  ImGuiDockNodeFlags_NoCloseButton |
                                      ImGuiDockNodeFlags_NoWindowMenuButton |
                                      ImGuiDockNodeFlags_DockSpace);

        ImGuiID id1, id2, id3, id4, id5, id6, id7, id8;
        ImGui::DockBuilderSetNodePos(dock_id, ImVec2(0, m_heightTabSection));
        ImGui::DockBuilderSetNodeSize(
            dock_id, ImVec2(width, height - m_heightTabSection));

        ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Right,
                                    preferredConsoleFrac, &id2, &id1);
        ImGui::DockBuilderSplitNode(id1, ImGuiDir_Left,
                                    preferredControlsWidth / float(width), &id3,
                                    &id4);
        ImGui::DockBuilderSplitNode(id3, ImGuiDir_Down, 0.5f, &id5, &id6);
        ImGui::DockBuilderSplitNode(id5, ImGuiDir_Down, 0.4f, &id7, &id8);
        ImGui::DockBuilderDockWindow("Console", id2);
        ImGui::DockBuilderDockWindow("Data", id8);
        ImGui::DockBuilderDockWindow("Properties", id7);
        ImGui::DockBuilderDockWindow("Controls", id6);
        ImGui::DockBuilderDockWindow("Viewer", id4);
        ImGui::DockBuilderFinish(dock_id);

        m_dockingNeedsReset = false;
    }

    // **** subroutine for async exection of heavy tasks
    if (m_startedHeavyCalculus) {
        static unsigned int counter = 0;
        static std::vector<std::string> chars{
            "[    ]", "[    ]", "[=   ]", "[==  ]", "[=== ]", "[ ===]",
            "[  ==]", "[   =]", "[    ]", "[   =]", "[  ==]", "[ ===]",
            "[====]", "[=== ]", "[==  ]", "[=   ]"};

        if (m_heavyAsyncTask.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
            counter++;
            ImGui::SetNextWindowSize(ImVec2(400, 200));
            ImGui::OpenPopup("Pollen3D");
            if (ImGui::BeginPopupModal("Pollen3D", nullptr,
                                       ImGuiWindowFlags_Modal |
                                           ImGuiWindowFlags_NoResize |
                                           ImGuiWindowFlags_NoSavedSettings)) {
                ImGui::SetCursorPos(ImVec2(135, 90));
                ImGui::Text("Please wait");
                ImGui::SameLine();

                if (m_fontMonoSmall) ImGui::PushFont(m_fontMonoSmall);
                char temp = (counter / 4) & 0x0F;
                ImGui::Text("%s", chars[temp].c_str());
                if (m_fontMonoSmall) ImGui::PopFont();

                ImGui::EndPopup();
            }
        } else {
            m_heavyAsyncTask.get();
            m_startedHeavyCalculus = false;
            ImGui::CloseCurrentPopup();
        }
    }

    _processKeyboardInput();

    ImGui::Render();
}

void Application::_drawMenuBar(int width)
{
    static bool showImGuiMetrics = false;
    static bool showDemoImGui = false;

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, m_heightTabSection),
                                        ImVec2(width, m_heightTabSection));
    ImGui::Begin("##tab-widget", nullptr,
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_MenuBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

    if (ImGui::BeginMenuBar()) {
        //        if (ImGui::BeginMenu("File")) {
        //            ImGui::MenuItem("Main menu bar");
        //            ImGui::MenuItem("Console");
        //            ImGui::MenuItem("Log");
        //            ImGui::MenuItem("Simple layout");
        //            ImGui::MenuItem("Property editor");
        //            ImGui::MenuItem("Long text display");
        //            ImGui::EndMenu();
        //        }
        if (ImGui::BeginMenu("Edit")) {
            ImGui::MenuItem("Undo", "Ctrl+Z");
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Reset docking", nullptr, &m_dockingNeedsReset);
            ImGui::Separator();
            ImGui::MenuItem("Show console", "Ctrl+T", &m_showConsole);
            ImGui::Separator();
            if (ImGui::MenuItem("Style dark")) ImGui::StyleColorsDark();
            if (ImGui::MenuItem("Style light")) ImGui::StyleColorsLight();
            ImGui::EndMenu();
        }

#ifdef POLLEN3D_DEBUG
        if (ImGui::BeginMenu("Debug")) {
            ImGui::MenuItem("Show metrics", nullptr, &showImGuiMetrics);
            ImGui::MenuItem("Show demo", nullptr, &showDemoImGui);
            ImGui::EndMenu();
        }
#endif

        ImGui::EndMenuBar();
    }

    int buttonH = 32;
    ImVec2 buttonSquare(buttonH, buttonH);
    ImVec2 buttonRect(0.0f, buttonH);

    if (ImGui::Button(ICON_FA_UPLOAD " Load images", buttonRect)) {
        auto files = loadImagesDialog();
        if (files.empty()) {
            LOG_ERR("Nothing to load");
        } else {
            auto f = [&](const std::vector<std::string> &files) {
                ProjectManager::get()->loadImages(&m_projectData, files);
                if (!m_projectData.empty()) m_currentImage = 0;
                _resetAppState();
            };
            _doHeavyTask(f, files);
        }
        m_currentTabForce = Tab_Image;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_SAVE " Save", buttonRect)) {
        std::string path = m_projectData.getProjectPath();
        if (path == "") path = saveProjectDialog();
        auto f = [&](const std::string &path) {
            ProjectManager::get()->saveProject(&m_projectData, path);
            _resetAppState();
        };
        _doHeavyTask(f, path);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save as...", buttonRect)) {
        auto file = saveProjectDialog();
        ProjectManager::get()->saveProject(&m_projectData, file);
        _resetAppState();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_FOLDER_OPEN " Open", buttonRect)) {
        auto file = openProjectDialog();
        LOG_DBG("Open project: %s", file.c_str());

        auto f = [&, file]() {
            ProjectManager::get()->openProject(&m_projectData, file);
            _resetAppState();
        };
        if (file != "") _doHeavyTask(f);
    }

    // ----- Close project popup

    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_FOLDER_MINUS " Close", buttonRect)) {
        ImGui::OpenPopup("Close project?");
    }

    if (ImGui::BeginPopupModal("Close project?", NULL,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text(
            "Are you sure you want to close the project?\n"
            "All the unsaved data will be lost.\n\n");

        if (ImGui::Button("OK", ImVec2(120, 0))) {
            LOG_DBG("Close project");
            ProjectManager::get()->closeProject(&m_projectData);
            _resetAppState();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // ***** Views

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(20, buttonH));
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_IMAGE "", buttonSquare)) {
        if (m_currentTab != Tab_Image) _resetAppState();
        m_currentTabForce = Tab_Image;
    }
    ImGuiC::HoveredTooltip("Set current view: image");

    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_IMAGES "", buttonSquare)) {
        if (m_currentTab != Tab_Stereo) _resetAppState();
        m_currentTabForce = Tab_Stereo;
    }
    ImGuiC::HoveredTooltip("Set current view: image pairs");
    ImGui::SameLine();

    if (ImGui::Button(ICON_FA_LAYER_GROUP "", buttonSquare)) {
        if (!isOneOf(m_currentTab, {Tab_Multiview, Tab_PointCloud}))
            _resetAppState();
        m_currentTabForce = Tab_Multiview;
    }
    ImGuiC::HoveredTooltip("Set current view: multiview");
    ImGui::SameLine();

    if (ImGui::Button(ICON_FA_CLOUD "", buttonSquare)) {
        if (!isOneOf(m_currentTab, {Tab_Multiview, Tab_PointCloud}))
            _resetAppState();
        m_currentTabForce = Tab_PointCloud;
    }
    ImGuiC::HoveredTooltip("Set current view: point cloud");

#ifdef POLLEN3D_DEBUG
    ImGui::SameLine();
    ImGui::Dummy(ImVec2(20, buttonH));
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, COLOR_GREEN);
    if (ImGui::Button("DBG_LOAD", buttonRect)) {
        std::vector<std::string> imPaths;
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_00.tif");
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_01.tif");
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_02.tif");
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_03.tif");
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_04.tif");
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_05.tif");
        imPaths.push_back(
            "/home/andrey/Projects/pollen3d/_datasets/pot_06.tif");

        auto f = [&, imPaths]() {
            ProjectManager::get()->loadImages(&m_projectData, imPaths);
            if (!m_projectData.empty()) m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_PROJ", buttonRect)) {
        auto f = [&]() {
            ProjectManager::get()->openProject(
                &m_projectData,
                "test_project" + std::string(P3D_PROJECT_EXTENSION));
            m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_PROJ2", buttonRect)) {
        auto f = [&]() {
            ProjectManager::get()->openProject(
                &m_projectData,
                "test_project2" + std::string(P3D_PROJECT_EXTENSION));
            m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_PROJ3", buttonRect)) {
        auto f = [&]() {
            ProjectManager::get()->openProject(
                &m_projectData,
                "test_project3" + std::string(P3D_PROJECT_EXTENSION));
            m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::PopStyleColor();
#endif

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(20, buttonH));
    ImGui::SameLine();
    if (m_projectData.nbImages() < 2) ImGuiC::PushDisabled();
    if (ImGui::Button(ICON_FA_ROCKET " One click!", buttonRect)) {
        LOG_OK("3D reconstruction in one click...");
        auto f = [&]() {
            ProjectManager::get()->extractFeatures(m_projectData);
            ProjectManager::get()->matchFeatures(m_projectData);
            ProjectManager::get()->findFundamentalMatrix(m_projectData);

            ProjectManager::get()->findMeasurementMatrixFull(m_projectData);
            ProjectManager::get()->findMeasurementMatrix(m_projectData);
            ProjectManager::get()->autocalibrate(m_projectData);

            ProjectManager::get()->triangulateSparse(m_projectData);
            ProjectManager::get()->bundleAdjustment(m_projectData);

            ProjectManager::get()->rectifyImagePairs(m_projectData);
            ProjectManager::get()->findDisparityMap(m_projectData);
            ProjectManager::get()->filterDisparityBilateral(m_projectData);

            ProjectManager::get()->triangulateDenseStereo(m_projectData, {0});

            _resetAppState();
        };
        _doHeavyTask(f);
    }
    if (m_projectData.nbImages() < 2) ImGuiC::PopDisabled();

    ImGui::End();

    if (showImGuiMetrics) ImGui::ShowMetricsWindow();
    if (showDemoImGui) ImGui::ShowDemoWindow();
}

void Application::_drawControls()
{
    if (!ImGui::BeginTabBar("##Tabs",
                            ImGuiTabBarFlags_NoCloseWithMiddleMouseButton))
        return;

    _drawTab_Image();
    _drawTab_Stereo();
    _drawTab_Multiview();
    _drawTab_PointCloud();

    ImGui::SameLine();
    const char *tabName = "";
    switch (m_currentTab) {
    case Tab_Image:
        tabName = "image";
        break;
    case Tab_Stereo:
        tabName = "stereo";
        break;
    case Tab_Multiview:
        tabName = "multiview";
        break;
    case Tab_PointCloud:
        tabName = "point cloud";
        break;
    }
    ImGui::Text("%s", tabName);

    ImGui::EndTabBar();
}

void Application::_drawTab_Image()
{
    ImGuiTabItemFlags flags = ImGuiTabItemFlags_NoCloseButton;
    if (m_currentTabForce == Tab_Image) {
        flags = ImGuiTabItemFlags_SetSelected;
        m_currentTabForce = -1;
    }

    if (ImGui::BeginTabItem(ICON_FA_IMAGE "", nullptr, flags)) {
        ImGui::Dummy(ImVec2(0.0f, 5.0f));

        // ***** Feature extraction (widget)

        if (m_widgetFeat) {
            m_widgetFeat->draw(m_projectData, m_currentImage);

            if (m_widgetFeat->isRequested("run")) {
                auto f = [&]() {
                    ProjectManager::get()->extractFeatures(m_projectData,
                                                           {m_currentImage});
                };
                _doHeavyTask(f);
            }
            if (m_widgetFeat->isRequested("run_all")) {
                auto f = [&]() {
                    ProjectManager::get()->extractFeatures(m_projectData);
                };
                _doHeavyTask(f);
            }
        }

        ImGui::EndTabItem();

        if (!isOneOf(m_currentTab, {Tab_General, Tab_Image})) _resetAppState();
        m_currentTab = Tab_Image;
    }
}

void Application::_drawTab_Stereo()
{
    ImGuiTabItemFlags flags = ImGuiTabItemFlags_NoCloseButton;
    if (m_currentTabForce == Tab_Stereo) {
        flags |= ImGuiTabItemFlags_SetSelected;
        m_currentTabForce = -1;
    }
    if (ImGui::BeginTabItem(ICON_FA_IMAGES "", nullptr, flags)) {
        if (ImGui::BeginChild("")) {
            ImGui::Dummy(ImVec2(0.0f, 5.0f));
            int matchingWidgetW = 250;

            bool disabled = false;
            ImagePair *imPair = m_projectData.imagePair(m_currentImage);
            if (!imPair) disabled = true;

            // ***** Matching (widget)

            if (m_widgetMatching) {
                m_widgetMatching->draw(m_projectData, m_currentImage);

                auto f = [&](const std::vector<int> &imIds) {
                    bool success =
                        ProjectManager::get()->matchFeatures(m_projectData, imIds);
                    if (success) {
                        m_currentSection = Section_Matches;
                        m_textureNeedsUpdate = true;
                    }
                };

                if (m_widgetMatching->isRequested("run"))
                    _doHeavyTask(f, std::vector<int>({m_currentImage}));
                if (m_widgetMatching->isRequested("run_all"))
                    _doHeavyTask(f, std::vector<int>());
            }

            // ***** Epipolar geometry

            {
                bool disableButtons =
                    disabled || !imPair || !imPair->hasMatches();
                if (disableButtons) ImGuiC::PushDisabled();

                bool run{false}, runAll{false};
                if (ImGuiC::Collapsing("Epipolar geometry", &run, &runAll)) {
                    ImGuiC::BeginSubGroup();
                    if (ImGui::Button(P3D_ICON_RUN " Find fundamental matrix"))
                        run = true;
                    ImGui::SameLine();
                    if (ImGui::Button(P3D_ICON_RUNALL " ALL##fundamental"))
                        runAll = true;
                    ImGuiC::EndSubGroup();
                }
                if (disableButtons) ImGuiC::PopDisabled();

                auto f = [&](const std::vector<int> &imIds) {
                    bool success = ProjectManager::get()->findFundamentalMatrix(
                        m_projectData, imIds);
                    if (success) {
                        m_currentSection = Section_Epilines;
                        m_textureNeedsUpdate = true;
                    }
                };

                if (run) _doHeavyTask(f, std::vector<int>({m_currentImage}));
                if (runAll) _doHeavyTask(f, std::vector<int>());
            }

            // ***** Rectification

            {
                bool disableButtons = disabled;
                if (!disableButtons) {
                    disableButtons |= utils::floatEq(imPair->getTheta1(), 0.0);
                    disableButtons |= utils::floatEq(imPair->getTheta2(), 0.0);
                }
                if (disableButtons) ImGuiC::PushDisabled();

                bool run{false}, runAll{false};
                if (ImGuiC::Collapsing("Rectification", &run, &runAll)) {
                    ImGuiC::BeginSubGroup();
                    if (ImGui::Button(P3D_ICON_RUN " Rectify image pair"))
                        run = true;
                    ImGui::SameLine();
                    if (ImGui::Button(P3D_ICON_RUNALL " ALL##rectify"))
                        runAll = true;
                    ImGuiC::EndSubGroup();
                }
                if (disableButtons) ImGuiC::PopDisabled();

                auto f = [&](const std::vector<int> &imIds) {
                    bool success =
                        ProjectManager::get()->rectifyImagePairs(m_projectData, imIds);
                    if (success) {
                        m_currentSection = Section_Rectified;
                        m_textureNeedsUpdate = true;
                    }
                };

                if (run) _doHeavyTask(f, std::vector<int>({m_currentImage}));
                if (runAll) _doHeavyTask(f, std::vector<int>());
            }

            // ***** Dense matching (widget)

            if (m_widgetDenseMatching) {
                m_widgetDenseMatching->draw(m_projectData, m_currentImage);

                auto fDisp = [&](const std::vector<int> &imIds) {
                    bool success =
                        ProjectManager::get()->findDisparityMap(m_projectData, imIds);
                    if (success) {
                        m_currentSection = Section_DisparityMap;
                        m_textureNeedsUpdate = true;
                    }
                };

                const auto &run = m_widgetDenseMatching->isRequested("run");
                const auto &runall = m_widgetDenseMatching->isRequested("run_all");
                if (run) _doHeavyTask(fDisp, std::vector<int>({m_currentImage}));
                if (runall) _doHeavyTask(fDisp, std::vector<int>());

                if (m_widgetDenseMatching->isRequested("run_bilateral")) {
                    _doHeavyTask([&]() {
                        ProjectManager::get()->filterDisparityBilateral(
                            m_projectData, {m_currentImage});
                        m_textureNeedsUpdate = true;
                    });
                }
                if (m_widgetDenseMatching->isRequested("run_bilateral_all")) {
                    auto f = [&]() {
                        ProjectManager::get()->filterDisparityBilateral(
                            m_projectData);
                        m_textureNeedsUpdate = true;
                    };
                    _doHeavyTask(f);
                }
                if (m_widgetDenseMatching->isRequested("run_filter_speckles")) {
                    _doHeavyTask([&]() {
                        ProjectManager::get()->filterDisparitySpeckles(
                            m_projectData, {m_currentImage});
                        m_textureNeedsUpdate = true;
                    });
                }
                if (m_widgetDenseMatching->isRequested(
                        "run_filter_speckles_all")) {
                    _doHeavyTask([&]() {
                        ProjectManager::get()->filterDisparitySpeckles(
                            m_projectData);
                        m_textureNeedsUpdate = true;
                    });
                }
            }
            ImGui::EndChild();
        }

        ImGui::EndTabItem();

        if (m_currentTab != Tab_Stereo) _resetAppState();
        m_currentTab = Tab_Stereo;
    }
}

void Application::_drawTab_Multiview()
{
    ImGuiTabItemFlags flags = ImGuiTabItemFlags_NoCloseButton;
    if (m_currentTabForce == Tab_Multiview) {
        flags |= ImGuiTabItemFlags_SetSelected;
        m_currentTabForce = -1;
    }

    if (ImGui::BeginTabItem(ICON_FA_LAYER_GROUP "", nullptr, flags)) {
        if (ImGui::BeginChild("")) {
            ImGui::Dummy(ImVec2(0.0f, 5.0f));

            bool disable = m_projectData.nbImagePairs() < 2;
            if (disable) ImGuiC::PushDisabled();

            // ***** Measurement matrix
            {
                bool run{false};
                if (ImGuiC::Collapsing("Measurement matrix", &run)) {
                    ImGuiC::BeginSubGroup();

                    if (ImGui::Button("Get full W")) {
                        auto f = [&]() {
                            ProjectManager::get()->findMeasurementMatrixFull(
                                m_projectData);
                        };
                        _doHeavyTask(f);
                    }
                    if (ImGui::Button(P3D_ICON_RUN " Get W")) {
                        auto f = [&]() {
                            ProjectManager::get()->findMeasurementMatrix(
                                m_projectData);
                        };
                        _doHeavyTask(f);
                    }
                    ImGuiC::EndSubGroup();
                }
                if (run)
                    _doHeavyTask([&]() {
                        ProjectManager::get()->findMeasurementMatrixFull(
                            m_projectData);
                        ProjectManager::get()->findMeasurementMatrix(
                            m_projectData);
                    });
            }

            // ***** Autocalibration
            {
                bool run{false};
                if (ImGuiC::Collapsing("Autocalibration", &run)) {
                    ImGuiC::BeginSubGroup();

                    if (ImGui::Button(P3D_ICON_RUN " Autocalibrate"))
                        run = true;

                    ImGuiC::EndSubGroup();
                }
                if (run)
                    _doHeavyTask([&]() {
                        ProjectManager::get()->autocalibrate(m_projectData);
                        m_viewer3dNeedsUpdate = true;
                    });
            }

            // ***** Triangulation
            {
                bool run{false};
                if (ImGuiC::Collapsing("Triangulation", &run)) {
                    ImGuiC::BeginSubGroup();
                    if (ImGui::Button(P3D_ICON_RUN " Triangulate (sparse)"))
                        run = true;

                    ImGui::Separator();
                    static int imPairIdx = 0;
                    ImGui::SliderInt("pair idx", &imPairIdx, 0,
                                     m_projectData.nbImagePairs() - 1);
                    if (ImGui::Button("Triangulate dense (stereo)")) {
                        auto f = [&](int imPairIdx) {
                            ProjectManager::get()->triangulateDenseStereo(m_projectData,
                                                                          {imPairIdx});
                            m_viewer3dNeedsUpdate = true;
                        };
                        _doHeavyTask(f, imPairIdx);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("ALL")) {
                        _doHeavyTask([&]() {
                            ProjectManager::get()->triangulateDenseStereo(m_projectData,
                                                                          {});
                            m_viewer3dNeedsUpdate = true;
                        });
                    }
#ifdef POLLEN3D_DEBUG
                    if (ImGui::Button("Triangulate dense (multi-view)")) {
                        _doHeavyTask([&]() {
                            ProjectManager::get()->triangulateDenseDev(m_projectData);
                            m_viewer3dNeedsUpdate = true;
                        });
                    }

                    if (ImGui::Button("Triangulate dense (dev)")) {
                        _doHeavyTask([&]() {
                            ProjectManager::get()->triangulateDenseDev(m_projectData);
                            m_viewer3dNeedsUpdate = true;
                        });
                    }
#endif
                    ImGuiC::EndSubGroup();
                }
                if (run)
                    _doHeavyTask([&]() {
                        ProjectManager::get()->triangulateSparse(m_projectData);
                        m_viewer3dNeedsUpdate = true;
                    });
            }

            // ***** Bundle adjustment
            {
                bool run{false};
                if (ImGuiC::Collapsing("Bundle adjustment", &run)) {
                    ImGuiC::BeginSubGroup();
                    if (ImGui::Button(P3D_ICON_RUN
                                      " Bundle adjustment (sparse)"))
                        run = true;
                    ImGuiC::EndSubGroup();
                }
                if (run)
                    _doHeavyTask([&]() {
                        ProjectManager::get()->bundleAdjustment(m_projectData);
                        m_viewer3dNeedsUpdate = true;
                    });
            }

            // ***** Analyze

            if (ImGuiC::Collapsing("Analyze")) {
                ImGuiC::BeginSubGroup();

                static bool showReprojectionErrorPlot = false;
                if (ImGui::Button("Plot reprojection error (sparse)"))
                    showReprojectionErrorPlot = true;

                if (showReprojectionErrorPlot)
                    plot::ReprojectionError(m_projectData,
                                            &showReprojectionErrorPlot, 600);

                ImGuiC::EndSubGroup();
            }

            if (disable) ImGuiC::PopDisabled();

            ImGui::EndChild();
        }

        ImGui::EndTabItem();

        if (!isOneOf(m_currentTab, {Tab_Multiview, Tab_PointCloud}))
            _resetAppState();
        m_currentTab = Tab_Multiview;
    }
}

void Application::_drawTab_PointCloud()
{
    ImGuiTabItemFlags flags = ImGuiTabItemFlags_NoCloseButton;
    if (m_currentTabForce == Tab_PointCloud) {
        flags |= ImGuiTabItemFlags_SetSelected;
        m_currentTabForce = -1;
    }

    if (ImGui::BeginTabItem(ICON_FA_CLOUD "", nullptr, flags)) {
        if (ImGui::BeginChild("")) {
            ImGui::Dummy(ImVec2(0.0f, 5.0f));
            if (ImGui::CollapsingHeader("Export", m_collapsingHeaderFlags)) {
                std::vector<std::string> list =
                    m_projectData.getPointCloudCtnr().getAllLabels();
                std::vector<const char *> listC;
                for (const auto &l : list) listC.push_back(l.c_str());
                static int pcdIdx = 0;
                if (pcdIdx > list.size() - 1) pcdIdx = 0;

                if (list.size() == 0) ImGuiC::PushDisabled();

                ImGuiC::BeginSubGroup();
                ImGui::Combo("point clouds", &pcdIdx, listC.data(),
                             listC.size());

                if (ImGui::Button("Export PLY")) {
                    std::string filepath = exportPointCloudDialog();
                    std::string label = list[pcdIdx];
                    LOG_DBG("Exporting pcd: %s", label.c_str());

                    auto f = [&](std::string label, std::string filepath) {
                        ProjectManager::get()->exportPLY(m_projectData, label,
                                                         filepath);
                    };

                    _doHeavyTask(f, label, filepath);
                }
                if (list.size() == 0) ImGuiC::PopDisabled();

                ImGuiC::EndSubGroup();
            }
            ImGui::EndChild();
        }
        ImGui::EndTabItem();

        if (!isOneOf(m_currentTab, {Tab_Multiview, Tab_PointCloud}))
            _resetAppState();
        m_currentTab = Tab_PointCloud;
    }
}

void Application::_drawData()
{
    if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {
        if (m_currentImage >= m_projectData.nbImages()) {
            m_currentImage = 0;
            _resetAppState();
        }

        if (ImGui::TreeNodeEx("Image list:", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (int n = 0; n < m_projectData.nbImages(); n++) {
                auto imPtr = m_projectData.image(n);
                if (!imPtr) continue;

                std::string entry = ICON_FA_IMAGE " " + imPtr->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n) {
                        m_currentImage = n;
                        m_currentSection = Section_Default;
                        m_textureNeedsUpdate = true;
                        LOG_DBG("Selection changed: %i", n);
                    }
                }
            }
            ImGui::TreePop();
        }
        return;
    }

    if (m_currentTab == Tab_Stereo) {
        if (m_currentImage >= m_projectData.nbImagePairs()) {
            m_currentImage = 0;
            m_textureNeedsUpdate = true;
        }

        if (ImGui::TreeNodeEx("Image pairs:", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (auto n = 0; n < (int)m_projectData.nbImagePairs(); n++) {
                auto imL = m_projectData.imagePairL(n);
                auto imR = m_projectData.imagePairR(n);
                if (!imL) continue;
                if (!imR) continue;

                std::string entry =
                    ICON_FA_IMAGES " " + imL->name() + " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n ||
                        !isOneOf(m_currentSection, {Section_Matches, Section_Epilines})) {
                        m_currentImage = n;
                        m_currentSection = Section_Matches;
                        m_textureNeedsUpdate = true;
                        LOG_DBG("Selection changed: %i", n);
                        LOG_DBG(" - section: %i", m_currentSection);
                    }
                }
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Rectified:", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (auto n = 0; n < (int)m_projectData.nbImagePairs(); n++) {
                auto imPair = m_projectData.imagePair(n);
                if (!imPair || !imPair->isRectified()) continue;

                auto imL = m_projectData.imagePairL(n);
                auto imR = m_projectData.imagePairR(n);
                if (!imL) continue;
                if (!imR) continue;

                std::string entry = ICON_FA_ALIGN_CENTER " " + imL->name() +
                                    " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n ||
                        m_currentSection != Section_Rectified) {
                        m_currentImage = n;
                        m_currentSection = Section_Rectified;
                        m_textureNeedsUpdate = true;
                        LOG_DBG("Selection changed: %i", n);
                        LOG_DBG(" - section: %i", m_currentSection);
                    }
                }
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Disparity:", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (auto n = 0; n < (int)m_projectData.nbImagePairs(); n++) {
                auto imPair = m_projectData.imagePair(n);
                if (!imPair || !imPair->hasDisparityMap()) continue;

                auto imL = m_projectData.imagePairL(n);
                auto imR = m_projectData.imagePairR(n);
                if (!imL) continue;
                if (!imR) continue;

                std::string entry = ICON_FA_ALIGN_CENTER " " + imL->name() +
                                    " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n ||
                        m_currentSection != Section_DisparityMap) {
                        m_currentImage = n;
                        m_currentSection = Section_DisparityMap;
                        m_textureNeedsUpdate = true;
                        LOG_DBG("Selection changed: %i", n);
                        LOG_DBG(" - section: %i", m_currentSection);
                    }
                }
            }
            ImGui::TreePop();
        }
        return;
    }

    if (m_currentTab == Tab_Multiview || m_currentTab == Tab_PointCloud) {
        if (ImGui::TreeNodeEx("Point clouds:",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
            auto &pcds = m_projectData.pointCloudCtnr();

            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 0.0f));
            auto w = ImGui::GetWindowWidth();
            ImGuiContext &g = *GImGui;
            float button_size = g.FontSize;
            float button1_x = w - g.Style.FramePadding.x * 2.0f - button_size;
            float button2_x =
                w - g.Style.FramePadding.x * 3.0f - 2.0f * button_size;
            ImGuiWindow *window = ImGui::GetCurrentWindow();

            for (auto &pcd : pcds) {
                std::string lblstr = pcd.getLabel();
                const char *lbl = lblstr.c_str();

                auto nbPts = pcd.nbPoints();
                auto visible = pcd.isVisible();

                ImGui::Text("%s (%i)", lbl, nbPts);
                float button_y = window->DC.LastItemRect.Min.y + 2;

                ImGuiID id = window->GetID(lbl);

                if (!visible) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5);
                if (ImGuiC::RunButton(window->GetID((void *)((intptr_t)id + 1)),
                                      ImVec2(button2_x, button_y),
                                      P3D_ICON_VISIBLE)) {
                    pcd.setVisible(!visible);
                    m_viewer3dNeedsUpdateVisibility = true;
                }
                if (!visible) ImGui::PopStyleVar();

                char popupLbl[128];
                ImFormatString(popupLbl, IM_ARRAYSIZE(popupLbl),
                               "Delete point cloud?##%s", lbl);

                if (ImGuiC::RunButton(window->GetID((void *)((intptr_t)id + 2)),
                                      ImVec2(button1_x, button_y),
                                      P3D_ICON_DELETE)) {
                    ImGui::OpenPopup(popupLbl);
                }

                if (ImGui::BeginPopupModal(popupLbl, NULL,
                                           ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text(
                        "Are you sure you want to delete the point cloud:");
                    ImGui::Text("- %s\n\n", lbl);

                    if (ImGui::Button("OK", ImVec2(120, 0))) {
                        ProjectManager::get()->deletePointCloud(m_projectData,
                                                                lbl);
                        m_viewer3dNeedsUpdate = true;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SetItemDefaultFocus();
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }
            }
            ImGui::PopStyleVar();
            ImGui::TreePop();
        }
        return;
    }
}

void Application::_drawProperties()
{
    ImGui::Columns(2, "mycolumns3", true);  // 2 cols with border

    if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {
        auto im = m_projectData.image(m_currentImage);
        if (!im) return;

        drawProperty_basic(im->getPath().c_str(), "path", "%s");
        drawProperty_basic(im->width(), "width", "%i");
        drawProperty_basic(im->height(), "height", "%i");
        drawProperty_basic(im->getNbFeatures(), "features", "%i");
        if (!im->getTranslation().isApprox(Vec2(0, 0))) {
            drawProperty_matrix(im->getCamera().getK(), "K",
                                "Calibration matrix (intrinsic parameters)");
            drawProperty_matrix(im->getTranslation(), "t",
                                "Camera translation (extrinsic parameters)");
        }
        return;
    }

    if (m_currentTab == Tab_Stereo) {
        auto imPair = m_projectData.imagePair(m_currentImage);
        if (!imPair) return;

        if (imPair->hasMatches())
            drawProperty_basic(imPair->getNbMatches(), "matches", "%i");

        if (imPair->hasF()) {
            drawProperty_matrix(imPair->getFundMat(), "F",
                                "Fundamental matrix");
            drawProperty_basic(utils::rad2deg(imPair->getTheta1()), "theta",
                               "%0.3f deg");
            drawProperty_basic(utils::rad2deg(imPair->getTheta2()), "theta'",
                               "%0.3f deg");
        }

        if (imPair->hasRrelative()) {
            drawProperty_basic(utils::rad2deg(imPair->getRho()), "rho",
                               "%0.3f deg");
            drawProperty_matrix(imPair->getRrelative(), "R",
                                "Relative rotation between images");
        }
        return;
    }

    if (m_currentTab == Tab_Multiview) {
        const auto &Wfull = m_projectData.getMeasurementMatrixFull();
        if (Wfull.rows() > 0 && Wfull.cols() > 0)
            drawProperty_matrix(Wfull, "Wf", "Full measurement matrix");

        const auto &W = m_projectData.getMeasurementMatrix();
        if (W.rows() > 0 && W.cols() > 0)
            drawProperty_matrix(W, "W", "Measurement matrix");

        const auto &P = m_projectData.getCameraMatricesMat();
        if (P.rows() > 0 && P.cols() > 0)
            drawProperty_matrix(P, "P", "Concatenated camera matrices");

        auto pcdContainer = m_projectData.getPointCloudCtnr();
        if (pcdContainer.contains("sparse")) {
            const auto &Xf = pcdContainer.at("sparse").getVertices();
            if (Xf.cols() > 0)
                drawProperty_matrix(Xf, "Xf", "3D points from Wf");
        }

        if (pcdContainer.contains("dense")) {
            const auto &Xf = pcdContainer.at("dense").getVertices();
            if (Xf.cols() > 0)
                drawProperty_matrix(Xf, "Xd", "Dense point cloud");
        }

        return;
    }

    ImGui::Columns(1);
}

template <typename Type>
void Application::drawProperty_basic(const Type &v, const std::string &name,
                                     const char *fmt, const char *icon)
{
    bool hovered = false;
    auto icone = (icon != nullptr) ? icon : ICON_FA_CUBE;
    ImGui::Text("%s %s", icone, name.c_str());
    if (ImGui::IsItemHovered()) hovered = true;
    ImGui::NextColumn();
    ImGui::Text(fmt, v);
    if (hovered || ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::Text(fmt, v);
        ImGui::EndTooltip();
    }
    ImGui::NextColumn();
}

template <typename Scalar, int SizeX, int SizeY>
void Application::drawProperty_matrix(
    const Eigen::Matrix<Scalar, SizeX, SizeY> &A, const std::string &name,
    const std::string &longName)
{
    static Eigen::IOFormat CleanFmt(5, 0, " ", "##", "", "");
    static Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision,
                                        Eigen::DontAlignCols, ", ", ", ", "",
                                        "", " << ", ";");
    static Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "",
                                     "", "[", "];");

    int rows = (SizeX == -1) ? static_cast<int>(A.rows()) : SizeX;
    int cols = (SizeY == -1) ? static_cast<int>(A.cols()) : SizeY;

    bool hovered = false;
    ImGui::Text("%s %s", ICON_FA_BORDER_ALL, name.c_str());
    std::string popupName = "##matrix_prop_popup" + name + longName;
    if (ImGui::IsItemHovered()) hovered = true;
    ImGui::OpenPopupOnItemClick(popupName.c_str(), 1);

    ImGui::NextColumn();
    ImGui::Text("[%ix%i]", rows, cols);
    if (hovered || ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();

        if (longName != "") ImGui::Text("%s\n", longName.c_str());

        int maxX = 12;
        int maxY = 12;
        if (rows > maxX || cols > maxY) {
            ImGui::PushStyleColor(ImGuiCol_Text, COLOR_WARN);
            ImGui::Text(ICON_FA_EXCLAMATION_TRIANGLE
                        " Matrix is too big to display. Showing only the %ix%i "
                        "top left corner\n\n",
                        maxX, maxY);
            ImGui::PopStyleColor();
        }
        ImGui::PushFont(m_fontMono);

        maxX = std::min(maxX, rows);
        maxY = std::min(maxY, cols);
        const auto &block = A.block(0, 0, maxX, maxY);
        std::stringstream ss;
        ss << block.format(CleanFmt);
        std::string output = ss.str();

        auto rows = utils::split(output, "##");
        for (const auto &row : rows) { ImGui::Text("%s", row.c_str()); }

        ImGui::PopFont();
        ImGui::EndTooltip();
    }
    if (ImGui::BeginPopupContextItem(popupName.c_str())) {
        if (ImGui::Selectable("Copy to clipboard: Eigen")) {
            std::stringstream ss;
            ss << A.format(CommaInitFmt);
            std::string output = ss.str();

            ImGui::LogToClipboard();
            ImGui::LogText("%s.setZero(%li;%li);\nM %s", name.c_str(), A.rows(),
                           A.cols(), output.c_str());
            LOG_OK("Matrix %s is copied to clipboard", name.c_str());
            ImGui::CloseCurrentPopup();
        }
        if (ImGui::Selectable("Copy to clipboard: Octave/MATLAB")) {
            std::stringstream ss;
            ss << A.format(OctaveFmt);
            std::string output = ss.str();

            ImGui::LogToClipboard();
            ImGui::LogText("%s = %s", name.c_str(), output.c_str());
            LOG_OK("Matrix %s is copied to clipboard", name.c_str());
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    ImGui::NextColumn();
}

void Application::_drawCentral()
{
    float width = ImGui::GetWindowContentRegionMax().x;
    float height = ImGui::GetWindowContentRegionMax().y;

    if (isOneOf(m_currentTab, {Tab_Multiview, Tab_PointCloud}) &&
        m_viewer3D != nullptr) {
        m_viewer3D->setSize(width, height);

        if (m_viewer3dNeedsUpdate) {
            m_viewer3D->init();
            m_viewer3D->deletePointCloudsAll();
            auto &pcds = m_projectData.getPointCloudCtnr();
            for (auto &pcd : pcds) {
                if (!pcd.isVisible()) continue;
                m_viewer3D->addPointCloud(pcd.getLabel(), pcd.getVertices(),
                                          pcd.getColors());
                LOG_DBG("Adding point cloud: %s (%i)", pcd.getLabel().c_str(),
                        pcd.getVertices().cols());
            }
            m_viewer3dNeedsUpdate = false;
            m_viewer3dNeedsUpdateVisibility = false;
        }

        if (m_viewer3dNeedsUpdateVisibility) {
            const auto &pcds = m_projectData.getPointCloudCtnr();
            for (const auto &pcd : pcds) {
                m_viewer3D->setPointCloudVisible(pcd.getLabel(),
                                                 pcd.isVisible());
            }
            m_viewer3dNeedsUpdateVisibility = false;
        }

        ImGui::PushFont(m_fontMonoSmall);
        m_viewer3D->draw(width, height);
        ImGui::PopFont();
        return;
    }

    if (isOneOf(m_currentTab, {Tab_General, Tab_Image, Tab_Stereo})) {
        static bool showFeatures = true;
        static bool showAnimated = false;
        int controlBottomBarHeight = 35;
        static float imageOpacity = 1.0f;
        static int skipEvery = 1;
        static float lineWidth = 1.0f;

        static float featuresSize = 2.5f;
        static ImVec4 color = COLOR_GREEN;

        if (m_textureNeedsUpdate) {
            cv::Mat matToBind;
            if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {
                auto imPtr = m_projectData.image(m_currentImage);
                if (imPtr) matToBind = imPtr->cvMat();
            } else if (m_currentTab == Tab_Stereo) {
                auto imPair = m_projectData.imagePair(m_currentImage);
                if (imPair && imPair->isValid()) {
                    if (isOneOf(m_currentSection, {Section_Matches, Section_Epilines})) {
                        auto imIdxL = imPair->imL();
                        auto imIdxR = imPair->imR();
                        auto imL = m_projectData.image(imIdxL);
                        auto imR = m_projectData.image(imIdxR);

                        if (imL && imR)
                            matToBind = utils::concatenateCvMat(
                                {imL->cvMat(), imR->cvMat()},
                                utils::CONCAT_HORIZONTAL);
                    } else if (m_currentSection == Section_Rectified) {
                        const auto &imL = imPair->getRectifiedImageL();
                        const auto &imR = imPair->getRectifiedImageR();
                        if (!imL.empty() && !imR.empty())
                            matToBind = utils::concatenateCvMat(
                                {imL, imR}, utils::CONCAT_HORIZONTAL);
                    } else if (m_currentSection == Section_DisparityMap) {
                        auto disp = imPair->getDisparityMap();
                        DenseMatchingUtil::getDispForPlot(disp, matToBind);
                    }
                }
            }

            if (!matToBind.empty())
                textureBind(matToBind);
            else {
                m_textureHeight = 0;
                m_textureWidth = 0;
            }
            m_textureNeedsUpdate = false;
        }

        if (isTextureReady()) {
            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - controlBottomBarHeight;
            float texAspect = m_textureHeight / float(m_textureWidth);
            float winAspect = height / float(width);

            float newW, newH;

            float imStartX = 0;
            float imStartY = 0;
            if (texAspect > winAspect) {
                newH = height;
                newW = newH / texAspect;
                imStartX = (width - newW) / 2.0f;
            } else {
                newW = width;
                newH = newW * texAspect;
                imStartY = (height - newH) / 2.0f;
            }

            ImGui::SetCursorPos(ImVec2(imStartX, imStartY));
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, imageOpacity);
            textureDisplay(ImVec2(newW, newH), ImVec2(0, 0), ImVec2(1, 1));
            ImGui::PopStyleVar();

            ImVec2 winPos = ImGui::GetWindowPos();
            const float posX = winPos.x + imStartX;
            const float posY = winPos.y + imStartY;

            if (isOneOf(m_currentTab, {Tab_General, Tab_Image})) {
                if (showFeatures)
                    _showFeatures(ImVec2(posX, posY), ImVec2(newW, newH), color,
                                  featuresSize);
            } else if (m_currentTab == Tab_Stereo) {
                if (m_currentSection == Section_Matches) {
                    _showMatches(ImVec2(posX, posY), ImVec2(newW, newH), color, lineWidth,
                                 skipEvery);
                } else if (m_currentSection == Section_Epilines) {
                    _showEpilines(ImVec2(posX, posY), ImVec2(newW, newH), color,
                                  lineWidth, skipEvery);
                } else if (m_currentSection == Section_Rectified) {
                    // just draw a horizontal line to judge the quality of
                    // rectification
                    if (ImGui::IsItemHovered()) {
                        ImDrawList *draw_list = ImGui::GetWindowDrawList();
                        auto y = ImGui::GetIO().MousePos.y;
                        draw_list->AddLine(ImVec2(posX, y),
                                           ImVec2(posX + newW, y),
                                           ImColor(200, 0, 0, 120), 2);
                    }
                }
            }
        } else {
            ImGui::Text("No image to display");
        }

        ImGui::PushFont(m_fontMonoSmall);
        ImGui::SetCursorPos(
            ImVec2(0, ImGui::GetWindowHeight() - controlBottomBarHeight));
        ImGui::Separator();
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("##image-opacity", &imageOpacity, 0.0f, 1.0f,
                           "opacity:%.2f");
        ImGui::SameLine();
        if (isOneOf(m_currentTab, {Tab_General, Tab_Image})) {
            ImGui::Text("--");
            ImGui::SameLine();
            ImGui::Checkbox("Show features", &showFeatures);
            ImGui::SameLine();
            ImGui::Text("--");
            ImGui::SameLine();
            ImGui::SliderFloat("##feature-size", &featuresSize, 1.0f, 15.0f,
                               "feat_size:%.1f");
        } else if (m_currentTab == Tab_Stereo) {
            if (isOneOf(m_currentSection, {Section_Matches, Section_Epilines})) {
                const int SHOW_MATCHES = 0;
                const int SHOW_EPILINES = 1;
                int display = m_currentSection == Section_Matches ? 0 : 1;

                ImGui::Text("--");
                ImGui::SameLine();
                ImGui::RadioButton("matches", &display, 0);
                ImGui::SameLine();
                ImGui::RadioButton("epilines", &display, 1);

                m_currentSection = display == 0 ? Section_Matches : Section_Epilines;
            }
            ImGui::SameLine();
            const char *sliderText =
                (skipEvery == 1) ? "show all" : "skip every:%i";
            ImGui::SliderInt("##percentage_of_features", &skipEvery, 1, 10,
                             sliderText);
            ImGui::SameLine();
            ImGui::SliderFloat("##lineWidth", &lineWidth, 1.0f, 5.0f,
                               "line width");
        }
        if (isOneOf(
                m_currentTab,
                {Tab_General, Tab_Image}) /* || (display == SHOW_MATCHES)*/) {
            ImGui::SameLine();
            ImGui::ColorEdit4(
                "color of features##3", (float *)&color,
                ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
            ImGui::SameLine();
        }
        ImGui::PopFont();
        return;
    }
}

void Application::_processKeyboardInput()
{
    const int ImGuiKey_UpArrow = 0x109;
    const int ImGuiKey_DownArrow = 0x108;
    const int ImGuiKey_Z = 0x57;
    const int ImGuiKey_Tab = 0x102;

    // ImGui::CaptureKeyboardFromApp(true);
    ImGuiIO &io = ImGui::GetIO();
    if (io.KeyCtrl) {
        if (ImGui::IsKeyPressed('s') || ImGui::IsKeyPressed('S')) {
            auto f = [&]() {
                std::string path = m_projectData.getProjectPath();
                if (path == "") path = saveProjectDialog();
                ProjectManager::get()->saveProject(&m_projectData, path);
            };
            _doHeavyTask(f);
        } else if (ImGui::IsKeyPressed('t') || ImGui::IsKeyPressed('T')) {
            m_showConsole = !m_showConsole;
        } else if (ImGui::IsKeyPressed(ImGuiKey_Z)) {
            m_textureNeedsUpdate = true;
            m_viewer3dNeedsUpdate = true;
            CommandManager::get()->undoCommand();
        } else if (ImGui::IsKeyPressed(ImGuiKey_Tab)) {
        }
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
        LOG_DBG("Up pressed");
        if (!isOneOf(m_currentTab, {Tab_General, Tab_Image, Tab_Stereo}))
            return;

        int modulo = m_projectData.nbImages();
        if (m_currentTab == Tab_Stereo) modulo = m_projectData.nbImagePairs();
        if (modulo > 0) {
            m_currentImage = (m_currentImage - 1 + modulo) % modulo;
            m_textureNeedsUpdate = true;
        }

    } else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
        LOG_DBG("Down pressed");

        if (!isOneOf(m_currentTab, {Tab_General, Tab_Image, Tab_Stereo}))
            return;

        int modulo = m_projectData.nbImages();
        if (m_currentTab == Tab_Stereo) modulo = m_projectData.nbImagePairs();

        if (modulo > 0) {
            m_currentImage = (m_currentImage + 1) % modulo;
            m_textureNeedsUpdate = true;
        }
    }
}

void Application::_showFeatures(const ImVec2 &pos, const ImVec2 &size,
                                const ImVec4 &col, float featuresSize)
{
    auto im = m_projectData.image(m_currentImage);
    if (im && im->hasFeatures()) {
        const auto &keyPts = im->getKeyPoints();
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        const auto color32 =
            IM_COL32(col.x * 255, col.y * 255, col.z * 255, col.w * 255);

        for (const auto &kpt : keyPts) {
            float x = pos.x + kpt.pt.x / float(m_textureWidth) * size.x;
            float y = pos.y + kpt.pt.y / float(m_textureHeight) * size.y;

            draw_list->AddCircle(ImVec2(x, y), featuresSize, color32, 6, 2);
        }
    }
}

void Application::_showEpilines(const ImVec2 &pos, const ImVec2 &size,
                                const ImVec4 &col, float lineWidth,
                                int skipEvery)
{
    std::vector<Vec2> ptsL, ptsR;
    m_projectData.getPairwiseMatches(m_currentImage, ptsL, ptsR);

    auto imPair = m_projectData.imagePair(m_currentImage);
    if (ptsL.empty() || ptsR.empty()) return;

    auto imL = m_projectData.image(imPair->imL());
    auto imR = m_projectData.image(imPair->imR());

    if (!imL || !imR) return;

    // the texture is rendered started in {pos} with the size {size}
    float ratioLeftTotal = imL->width() / float(imL->width() + imR->width());
    float ratioRightTotal = (1.0f - ratioLeftTotal);

    if (imL->height() != imR->height()) {
        LOG_ERR(
            "Showing epilines is not supported yet for images with different "
            "height");
    }

    const float posXr = ratioLeftTotal * size.x;

    std::vector<Vec3> epilinesL, epilinesR;
    imPair->getEpilinesLeft(ptsR, epilinesL);
    imPair->getEpilinesRight(ptsL, epilinesR);

    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    auto mapToTextureL = [&](const Vec2 &pt) -> ImVec2 {
        return ImVec2(pos.x + float(pt[0]) / float(imL->width()) *
                                  ratioLeftTotal * size.x,
                      pos.y + float(pt[1]) / float(imL->height()) * size.y);
    };

    auto mapToTextureR = [&](const Vec2 &pt) -> ImVec2 {
        return ImVec2(
            pos.x + posXr +
                float(pt[0]) / float(imR->width()) * ratioRightTotal * size.x,
            pos.y + float(pt[1]) / float(imR->height()) * size.y);
    };

    static int hoveredLine = -1;
    bool wasHoveredThisFrame = false;
    float radius = 1.5f + lineWidth * 1.5f;
    for (int i = 0; i < ptsL.size(); i += skipEvery) {
        const auto &ptL = ptsL[i];
        const auto &ptR = ptsR[i];

        auto epilinePtsL =
            utils::lineIntersectBox(epilinesL[i], imL->width(), imL->height());
        auto epilinePtsR =
            utils::lineIntersectBox(epilinesR[i], imR->width(), imR->height());

        ImU32 color32 = IM_COL32(125, 125, 125, 125);
        if (hoveredLine == -1) {
            Vec3f color = palette::color1(i / float(ptsL.size()));
            color32 =
                IM_COL32(color(0) * 255, color(1) * 255, color(2) * 255, 255);
        }

        ImVec2 leftPoint = mapToTextureL(ptL);
        ImVec2 rightPoint = mapToTextureR(ptR);

        std::string featureLblL = "##featureL-" + std::to_string(i);
        std::string featureLblR = "##featureR-" + std::to_string(i);
        ImGuiC::ItemCircle(featureLblL.c_str(), leftPoint, radius, color32);
        if (ImGui::IsItemHovered()) {
            hoveredLine = i;
            wasHoveredThisFrame = true;
        }
        ImGuiC::ItemCircle(featureLblR.c_str(), rightPoint, radius, color32);
        if (ImGui::IsItemHovered()) {
            hoveredLine = i;
            wasHoveredThisFrame = true;
        }

        //        draw_list->AddCircle(leftPoint, 2, color32, 6, 3);
        //        draw_list->AddCircle(rightPoint, 2, color32, 6, 3);
        draw_list->AddLine(mapToTextureL(epilinePtsL.first),
                           mapToTextureL(epilinePtsL.second), color32,
                           lineWidth);
        draw_list->AddLine(mapToTextureR(epilinePtsR.first),
                           mapToTextureR(epilinePtsR.second), color32,
                           lineWidth);
    }

    if (hoveredLine != -1) {
        const auto &ptL = ptsL[hoveredLine];
        const auto &ptR = ptsR[hoveredLine];
        auto epilinePtsL = utils::lineIntersectBox(epilinesL[hoveredLine],
                                                   imL->width(), imL->height());
        auto epilinePtsR = utils::lineIntersectBox(epilinesR[hoveredLine],
                                                   imR->width(), imR->height());

        Vec3f color = palette::color1(hoveredLine / float(ptsL.size()));
        ImU32 color32 =
            IM_COL32(color(0) * 255, color(1) * 255, color(2) * 255, 255);
        draw_list->AddCircle(mapToTextureL(ptL), radius, color32, 6, 2);
        draw_list->AddCircle(mapToTextureR(ptR), radius, color32, 6, 2);
        draw_list->AddLine(mapToTextureL(epilinePtsL.first),
                           mapToTextureL(epilinePtsL.second), color32,
                           lineWidth);
        draw_list->AddLine(mapToTextureR(epilinePtsR.first),
                           mapToTextureR(epilinePtsR.second), color32,
                           lineWidth);
    }

    if (!wasHoveredThisFrame) hoveredLine = -1;
}

void Application::_showMatches(const ImVec2 &pos, const ImVec2 &size,
                               const ImVec4 &col, float lineWidth,
                               int skipEvery)
{
    std::vector<Vec2> ptsL, ptsR;
    m_projectData.getPairwiseMatches(m_currentImage, ptsL, ptsR);

    auto imPair = m_projectData.imagePair(m_currentImage);
    if (ptsL.empty() || ptsR.empty()) return;

    auto imL = m_projectData.image(imPair->imL());
    auto imR = m_projectData.image(imPair->imR());

    if (!imL || !imR) return;

    // the texture is rendered started in {pos} with the size {size}
    float ratioLeftTotal = imL->width() / float(imL->width() + imR->width());
    float ratioRightTotal = (1.0f - ratioLeftTotal);

    if (imL->height() != imR->height()) {
        LOG_ERR(
            "Showing epilines is not supported yet for images with different "
            "height");
    }

    const float posXr = ratioLeftTotal * size.x;

    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    auto mapToTextureL = [&](const Vec2 &pt) -> ImVec2 {
        return ImVec2(pos.x + float(pt[0]) / float(imL->width()) *
                                  ratioLeftTotal * size.x,
                      pos.y + float(pt[1]) / float(imL->height()) * size.y);
    };

    auto mapToTextureR = [&](const Vec2 &pt) -> ImVec2 {
        return ImVec2(
            pos.x + posXr +
                float(pt[0]) / float(imR->width()) * ratioRightTotal * size.x,
            pos.y + float(pt[1]) / float(imR->height()) * size.y);
    };

    static int hoveredLine = -1;
    bool wasHoveredThisFrame = false;
    float radius = 1.5f + lineWidth * 1.5f;
    for (int i = 0; i < ptsL.size(); i += skipEvery) {
        const auto &ptL = ptsL[i];
        const auto &ptR = ptsR[i];

        ImU32 color32 = IM_COL32(125, 125, 125, 125);
        if (hoveredLine == -1) {
            Vec3f color = palette::color1(i / float(ptsL.size()));
            color32 =
                IM_COL32(color(0) * 255, color(1) * 255, color(2) * 255, 255);
        }

        ImVec2 leftPoint = mapToTextureL(ptL);
        ImVec2 rightPoint = mapToTextureR(ptR);

        std::string featureLblL = "##featureL-" + std::to_string(i);
        std::string featureLblR = "##featureR-" + std::to_string(i);
        ImGuiC::ItemCircle(featureLblL.c_str(), leftPoint, radius, color32);
        if (ImGui::IsItemHovered()) {
            hoveredLine = i;
            wasHoveredThisFrame = true;
        }
        ImGuiC::ItemCircle(featureLblR.c_str(), rightPoint, radius, color32);
        if (ImGui::IsItemHovered()) {
            hoveredLine = i;
            wasHoveredThisFrame = true;
        }

        // draw_list->AddCircle( leftPoint, 2, color32, 6, 1 + lineWidth
        // * 1.5f); draw_list->AddCircle(rightPoint, 2, color32, 6, 1 +
        // lineWidth
        // * 1.5f);
        draw_list->AddLine(mapToTextureL(ptL), mapToTextureR(ptR), color32,
                           lineWidth);
    }

    if (hoveredLine != -1) {
        const auto &ptL = ptsL[hoveredLine];
        const auto &ptR = ptsR[hoveredLine];
        Vec3f color = palette::color1(hoveredLine / float(ptsL.size()));
        ImU32 color32 =
            IM_COL32(color(0) * 255, color(1) * 255, color(2) * 255, 255);
        draw_list->AddCircle(mapToTextureL(ptL), radius, color32, 6, 2);
        draw_list->AddCircle(mapToTextureR(ptR), radius, color32, 6, 2);
        draw_list->AddLine(mapToTextureL(ptL), mapToTextureR(ptR), color32,
                           lineWidth);
    }

    if (!wasHoveredThisFrame) hoveredLine = -1;
}
