#include "app.h"

#include <future>

#include "p3d/project_manager.h"
#include "p3d/commands.h"
#include "p3d/gui/palette.h"
#include "p3d/gui/imgui_custom.h"

#include "fonts/IconsFontAwesome5.h"

#ifndef P3D_PROJECT_EXTENSION
#define P3D_PROJECT_EXTENSION ".yml.gz"
#endif

void Application::initImGui(){
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;

    std::string pathToFonts = "../../3rdparty/fonts/";
    io.Fonts->AddFontFromFileTTF(std::string(pathToFonts + "Ubuntu-R.ttf").c_str(), 16.0f);

    static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
    ImFontConfig icons_config; icons_config.MergeMode = true; icons_config.PixelSnapH = true;
    io.Fonts->AddFontFromFileTTF( std::string(pathToFonts + FONT_ICON_FILE_NAME_FAS).c_str(), 16.0f, &icons_config, icons_ranges );

    m_fontMono = io.Fonts->AddFontFromFileTTF(std::string(pathToFonts + "UbuntuMono-R.ttf").c_str(), 16.0f);
    ConsoleLogger::get()->setFont(m_fontMono);
    m_fontMonoSmall = io.Fonts->AddFontFromFileTTF(std::string(pathToFonts + "UbuntuMono-R.ttf").c_str(), 14.0f);

    applyStyle();
}

void Application::applyStyle() {
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.FrameRounding  = 4;
    style.WindowRounding = 0;
    style.WindowBorderSize = 1;
    style.IndentSpacing = 8.0f;
    style.WindowMenuButtonPosition = ImGuiDir_Right;
    style.Colors[ImGuiCol_TitleBg] = style.Colors[ImGuiCol_TitleBgActive];

    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.11f,0.11f,0.11f,1.0f);
    style.Colors[ImGuiCol_ChildBg] = ImVec4(0.145f,0.145f,0.145f,1.0f);
    style.Colors[ImGuiCol_TitleBg] = ImVec4(0.2f,0.2f,0.2f,1.0f);
    style.Colors[ImGuiCol_TitleBgActive] = style.Colors[ImGuiCol_TitleBg];
    style.Colors[ImGuiCol_TabUnfocusedActive] = style.Colors[ImGuiCol_TitleBg];
    style.Colors[ImGuiCol_Header] = ImVec4(0.216f,0.216f,0.239f,1.0f);
    style.Colors[ImGuiCol_Tab] = ImVec4(0.33f, 0.33f, 0.33f, 0.86f);


}

void Application::draw(int width, int height){

    ImGui::NewFrame();

    _drawMenuBar(width);

    ImVec4 colorBg2 = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    colorBg2.w = 1.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);

    ImGui::SetNextWindowPos(ImVec2(0, m_heightTabSection));
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, height - m_heightTabSection),ImVec2(width, height - m_heightTabSection));
    ImGuiID dock_id = ImGui::GetID("ID().c_str()");
    if (ImGui::Begin("CentralArea",nullptr,ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        ImGui::DockSpace(dock_id);
        ImGui::End();
    }

    ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoBringToFrontOnFocus;

    if (ImGui::Begin("Controls",nullptr, flags)){
        _drawControls();
        ImGui::End();
    }

    ImGui::Begin("Data",nullptr, flags);
    _drawData();
    ImGui::End();

    ImGui::Begin("Properties",nullptr, flags);
    _drawProperties();
    ImGui::End();

    ImGui::PushStyleColor(ImGuiCol_ChildBg, colorBg2);
    ImGui::Begin("Viewer",nullptr, flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    _drawCentral();
    ImGui::End();
    ImGui::PopStyleColor();

    ImGui::Begin("Console",nullptr, flags);
    ConsoleLogger::get()->render();
    ImGui::End();

    ImGui::PopStyleVar();

    if (m_dockingNeedsReset){
        ImGuiID dock_id = ImGui::GetID("ID().c_str()");

        ImGui::DockBuilderRemoveNode(dock_id); // Clear out existing layout
        ImGui::DockBuilderAddNode(dock_id, // Add empty node
                                  ImGuiDockNodeFlags_NoCloseButton |
                                  ImGuiDockNodeFlags_NoWindowMenuButton);

        ImGuiID id1, id2, id3, id4, id5, id6, id7, id8;
        ImGui::DockBuilderSetNodePos(dock_id, ImVec2(0, m_heightTabSection));
        ImGui::DockBuilderSetNodeSize(dock_id, ImVec2(width, height - m_heightTabSection));
        ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Right,0.35f,&id2,&id1);
        ImGui::DockBuilderSplitNode(id1, ImGuiDir_Left,0.35f,&id3,&id4);
        ImGui::DockBuilderSplitNode(id3, ImGuiDir_Down,0.5f,&id5,&id6);
        ImGui::DockBuilderSplitNode(id5, ImGuiDir_Down,0.4f,&id7,&id8);
        ImGui::DockBuilderDockWindow("Console", id2);
        ImGui::DockBuilderDockWindow("Data", id8);
        ImGui::DockBuilderDockWindow("Properties", id7);
        ImGui::DockBuilderDockWindow("Controls", id6);
        ImGui::DockBuilderDockWindow("Viewer", id4);

        ImGui::DockBuilderGetNode(id2)->HasCloseButton = false;
        ImGui::DockBuilderGetNode(id2)->EnableCloseButton = false;
        ImGui::DockBuilderGetNode(id2)->HasWindowMenuButton = false;
        ImGui::DockBuilderGetNode(id3)->HasCloseButton = false;
        ImGui::DockBuilderGetNode(id4)->HasCloseButton = false;
        ImGui::DockBuilderFinish(dock_id);

        m_dockingNeedsReset = false;
    }

//        _renderLeftWidget();

//        ImGui::NextColumn();

//        _renderCentralWidget();

//        ImGui::NextColumn();
 //       ConsoleLogger::get()->render(1);
//        ImGui::NextColumn();
//        ImGui::EndColumns();
//        ImGui::End();
//    }

    // **** subroutine for async exection of heavy tasks
    if (m_startedHeavyCalculus) {
        static unsigned int counter = 0;
        static std::vector<std::string> chars{"[    ]",
                                              "[    ]",
                                              "[=   ]",
                                              "[==  ]",
                                              "[=== ]",
                                              "[ ===]",
                                              "[  ==]",
                                              "[   =]",
                                              "[    ]",
                                              "[   =]",
                                              "[  ==]",
                                              "[ ===]",
                                              "[====]",
                                              "[=== ]",
                                              "[==  ]",
                                              "[=   ]"};

        if (m_heavyAsyncTask.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            counter++;
            ImGui::SetNextWindowSize(ImVec2(400,200));
            ImGui::OpenPopup("Pollen3D");
            if (ImGui::BeginPopupModal("Pollen3D", nullptr,ImGuiWindowFlags_Modal | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings)) {
                ImGui::SetCursorPos(ImVec2(135,90));
                ImGui::Text("Please wait");
                ImGui::SameLine();

                if (m_fontMonoSmall) ImGui::PushFont(m_fontMonoSmall);
                char temp = (counter / 4) & 0x0F;
                ImGui::Text("%s",chars[temp].c_str());
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
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, m_heightTabSection),ImVec2(width, m_heightTabSection));
    ImGui::Begin("##tab-widget",nullptr,
         ImGuiWindowFlags_NoDocking |
         ImGuiWindowFlags_NoResize |
         ImGuiWindowFlags_NoCollapse |
         ImGuiWindowFlags_NoTitleBar |
         ImGuiWindowFlags_MenuBar |
         ImGuiWindowFlags_NoBringToFrontOnFocus);

    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            ImGui::MenuItem("Main menu bar");
            ImGui::MenuItem("Console");
            ImGui::MenuItem("Log");
            ImGui::MenuItem("Simple layout");
            ImGui::MenuItem("Property editor");
            ImGui::MenuItem("Long text display");
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Reset docking",nullptr,&m_dockingNeedsReset);
            ImGui::Separator();
            if(ImGui::MenuItem("Style dark")) ImGui::StyleColorsDark();
            if(ImGui::MenuItem("Style light")) ImGui::StyleColorsLight();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Examples"))
        {
            ImGui::MenuItem("Main menu bar");
            ImGui::MenuItem("Console");
            ImGui::MenuItem("Log");
            ImGui::MenuItem("Simple layout");
            ImGui::MenuItem("Property editor");
            ImGui::MenuItem("Long text display");
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Debug"))
        {
            ImGui::MenuItem("Show metrics", nullptr, &showImGuiMetrics);
            ImGui::MenuItem("Show demo", nullptr, &showDemoImGui);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    int buttonH = 32;
    ImVec2 buttonSquare(buttonH,buttonH);
    ImVec2 buttonRect(0.0f, buttonH);

    if (ImGui::Button(ICON_FA_UPLOAD" Load images",buttonRect))
    {
        auto files = ProjectManager::get()->loadImagesDialog();
        auto f = [&,files]() {
            ProjectManager::get()->loadImages(&m_projectData,files);
            if (!m_projectData.empty()) m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_SAVE" Save",buttonRect))
    {
        auto f = [&]() {
            ProjectManager::get()->saveProject(&m_projectData,m_projectData.getProjectPath());
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save as...",buttonRect))
    {
        auto file = ProjectManager::get()->saveProjectDialog();
        ProjectManager::get()->saveProject(&m_projectData, file);
        _resetAppState();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_FOLDER_OPEN" Open",buttonRect))
    {
        auto file = ProjectManager::get()->openProjectDialog();
        LOG_DBG("Open project: %s", file.c_str());

        auto f = [&,file]() {
            ProjectManager::get()->openProject(&m_projectData, file);
            _resetAppState();
        };
        if (file != "") _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_FOLDER_MINUS" Close",buttonRect))
    {
        LOG_DBG("Close project");
        ProjectManager::get()->closeProject(&m_projectData);
        _resetAppState();
    }
    ImGui::SameLine();

    /*
    if (ImGui::Button(ICON_FA_IMAGE"",buttonSquare)) {
        if (m_currentTab != Tab_Image) _resetAppState();
        m_currentTab = Tab_Image;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_IMAGES"",buttonSquare)) {
        if (m_currentTab != Tab_Stereo) _resetAppState();
        m_currentTab = Tab_Stereo;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_LAYER_GROUP"",buttonSquare)) {
        if (!isOneOf(m_currentTab, {Tab_Multiview,Tab_PointCloud})) _resetAppState();
        m_currentTab = Tab_Multiview;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_FA_CLOUD"",buttonSquare)) {
        if (!isOneOf(m_currentTab, {Tab_Multiview,Tab_PointCloud})) _resetAppState();
        m_currentTab = Tab_PointCloud;
    }
    ImGui::SameLine();
    */

    ImGui::PushStyleColor(ImGuiCol_Button,COLOR_GREEN);
    if (ImGui::Button("DBG_LOAD",buttonRect))
    {
        std::vector<std::string> imPaths;
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_00.tif");
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_01.tif");
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_02.tif");
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_03.tif");
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_04.tif");
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_05.tif");
        imPaths.push_back("/home/andrey/Projects/pollen3d/_datasets/pot_06.tif");

        auto f = [&,imPaths]() {
            ProjectManager::get()->loadImages(&m_projectData,imPaths);
            if (!m_projectData.empty()) m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_PROJ",buttonRect))
    {
        auto f = [&]() {
            ProjectManager::get()->openProject(&m_projectData, "test_project" + std::string(P3D_PROJECT_EXTENSION));
            m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_PROJ2",buttonRect))
    {
        auto f = [&]() {
            ProjectManager::get()->openProject(&m_projectData, "test_project2" + std::string(P3D_PROJECT_EXTENSION));
            m_currentImage = 0;
            _resetAppState();
        };
        _doHeavyTask(f);
    }
    ImGui::PopStyleColor();

    ImGui::End();

    if (showImGuiMetrics) ImGui::ShowMetricsWindow();
    if (showDemoImGui) ImGui::ShowDemoWindow();
}

void Application::_drawControls()
{
    if (!ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_None))
        return;

    _drawTab_Image();
    _drawTab_Stereo();
    _drawTab_Multiview();

    if (ImGui::BeginTabItem(ICON_FA_CLOUD""))
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1,0,0,1));
        ImGui::Text("May come soon ;)");
        ImGui::PopStyleColor();
        ImGui::EndTabItem();
        if (!isOneOf(m_currentTab, {Tab_Multiview,Tab_PointCloud})) _resetAppState();
        m_currentTab = Tab_PointCloud;
    }
    ImGui::EndTabBar();

    ImGui::SameLine();
    const char * tabName = "";
    switch (m_currentTab) {
        case Tab_Image: tabName = "image"; break;
        case Tab_Stereo: tabName = "stereo"; break;
        case Tab_Multiview: tabName = "multiview"; break;
        case Tab_PointCloud: tabName = "point cloud"; break;
    }
    ImGui::Text("%s",tabName);
}

void Application::_drawTab_Image()
{
    if (ImGui::BeginTabItem(ICON_FA_IMAGE""))
    {
        bool disabled = false;
        auto image = m_projectData.image(m_currentImage);
        if (image == nullptr) disabled = true;

        if (disabled) ImGuiC::PushDisabled();

        if (ImGui::CollapsingHeader("Feature extraction",m_collapsingHeaderFlags))
        {
            if (ImGui::Button("Extract features"))
            {
                auto f = [&]() {
                    ProjectManager::get()->extractFeatures(m_projectData, {m_currentImage});
                };
                _doHeavyTask(f);
            }
            if (ImGui::Button("Extract features ALL"))
            {
                auto f = [&]() {
                    ProjectManager::get()->extractFeatures(m_projectData);
                };
                _doHeavyTask(f);
            }
        }
        if (disabled) ImGuiC::PopDisabled();

        ImGui::EndTabItem();

        if (!isOneOf(m_currentTab, {Tab_General,Tab_Image})) _resetAppState();
        m_currentTab = Tab_Image;
    }
}

void Application::_drawTab_Stereo()
{
    if (ImGui::BeginTabItem(ICON_FA_IMAGES""))
    {
        int matchingWidgetW = 250;

        bool disabled = false;
        ImagePair * imPair = m_projectData.imagePair(m_currentImage);
        if (!imPair) disabled = true;

        if (ImGui::CollapsingHeader("Matching",m_collapsingHeaderFlags))
        {
            const char* matchingAlgos[] = { "BruteForce-L1", "BruteForce", "FlannBased"};
            auto matcherCurAlg     = ProjectManager::get()->getSetting(p3dSetting_matcherCurAlg).cast<int>();

            if (ImGui::Combo("matcher", &matcherCurAlg, matchingAlgos, IM_ARRAYSIZE(matchingAlgos))) {
                ProjectManager::get()->setSetting(p3dSetting_matcherCurAlg,matcherCurAlg);
            }

            {
                auto matcherFilterCoef = ProjectManager::get()->getSetting(p3dSetting_matcherFilterCoef).cast<float>();
                float min = 0.1f;
                float max = 1.0f;
                ImGui::InputFloat("filter coef", &matcherFilterCoef, min, max, "%.2f");
                matcherFilterCoef = std::max(min,std::min(matcherFilterCoef,max));
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setSetting(p3dSetting_matcherFilterCoef,matcherFilterCoef);
            }

            bool disableButtons = disabled;
            auto imL = m_projectData.imagePairL(m_currentImage);
            auto imR = m_projectData.imagePairR(m_currentImage);
            disableButtons = !((imL != nullptr) && imL->hasFeatures());
            disableButtons = !((imR != nullptr) && imR->hasFeatures());

            if (disableButtons) ImGuiC::PushDisabled();
            if (ImGui::Button("Match features"))
            {
                auto f = [&]() {
                    ProjectManager::get()->matchFeatures(m_projectData, {m_currentImage});
                };
                _doHeavyTask(f);
            }

            ImGui::SameLine();
            if (ImGui::Button("ALL##matching"))
            {
                auto f = [&]() {
                    ProjectManager::get()->matchFeatures(m_projectData);
                };
                _doHeavyTask(f);
            }
            if (disableButtons) ImGuiC::PopDisabled();
        }

        if (ImGui::CollapsingHeader("Epipolar geometry",m_collapsingHeaderFlags))
        {
            bool disableButtons = disabled || !imPair->hasMatches();
            if (disableButtons) ImGuiC::PushDisabled();

            if (ImGui::Button("Find fundamental matrix"))
            {
                auto f = [&]() {
                    ProjectManager::get()->findFundamentalMatrix(m_projectData, {m_currentImage});
                };
                _doHeavyTask(f);
            }

            ImGui::SameLine();
            if (ImGui::Button("ALL##fundamental"))
            {
                auto f = [&]() {
                    ProjectManager::get()->findFundamentalMatrix(m_projectData);
                };
                _doHeavyTask(f);
            }
            if (disableButtons) ImGuiC::PopDisabled();
        }

        if (ImGui::CollapsingHeader("Rectification",m_collapsingHeaderFlags))
        {
            bool disableButtons = disabled || !imPair->hasF();
            if (disableButtons) ImGuiC::PushDisabled();

            if (ImGui::Button("Rectify image pair"))
            {
                auto f = [&]() {
                    ProjectManager::get()->rectifyImagePairs(m_projectData, {m_currentImage});
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }

            ImGui::SameLine();
            if (ImGui::Button("ALL##rectify"))
            {
                auto f = [&]() {
                    ProjectManager::get()->rectifyImagePairs(m_projectData);
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            if (disableButtons) ImGuiC::PopDisabled();
        }

        if (ImGui::CollapsingHeader("Dense matching",m_collapsingHeaderFlags))
        {
            bool disableButtons = disabled || !imPair->hasF();
            if (disableButtons) ImGuiC::PushDisabled();

            const char* denseMatchingAlgos[] = { "SGBM", "HH", "SGBM_3WAY"};
            auto matcherCurAlg = imPair ? imPair->denseMatching.dispMethod : 0;

            if (ImGui::Combo("method", &matcherCurAlg, denseMatchingAlgos, IM_ARRAYSIZE(denseMatchingAlgos))) {
                ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispMethod,matcherCurAlg);
            }

            {
                auto dispLowerBound = imPair ? imPair->denseMatching.dispLowerBound : -1;
                auto dispUpperBound = imPair ? imPair->denseMatching.dispUpperBound : 2;
                auto dispBlockSize  = imPair ? imPair->denseMatching.dispBlockSize  : 9;

                ImGui::InputInt("lowew bound",&dispLowerBound);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispLowerBound,dispLowerBound);
                ImGui::InputInt("upper bound",&dispUpperBound);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispUpperBound,dispUpperBound);
                ImGui::InputInt("block size",&dispBlockSize,2,2);
                if (ImGui::IsItemEdited()) {
                    if (dispBlockSize % 2 == 0) dispBlockSize--;
                    dispBlockSize = std::min(dispBlockSize,21);
                    dispBlockSize = std::max(dispBlockSize,1);
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispBlockSize,dispBlockSize);
                }
            }

            if (ImGui::Button("Dense match"))
            {
                auto f = [&]() {
                    ProjectManager::get()->findDisparityMap(m_projectData, {m_currentImage});
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            ImGui::SameLine();
            if (ImGui::Button("ALL##densematch"))
            {
                auto f = [&]() {
                    ProjectManager::get()->findDisparityMap(m_projectData);
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            ImGui::Separator();

            ImGui::Text("Bilateral filter: ");
            {
                auto v1 = imPair ? imPair->denseMatching.bilateralD : 9;
                auto v2 = imPair ? imPair->denseMatching.bilateralSigmaColor : 180;
                auto v3 = imPair ? imPair->denseMatching.bilateralSigmaSpace  : 180;

                ImGui::InputInt("diameter",&v1);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_BilateralD,v1);
                helpMarker("Diameter of each pixel neighborhood that is used during filtering. "
                           "If it is non-positive, it is computed from sigmaSpace.");

                ImGui::InputInt("sigma color",&v2);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_BilateralSigmaColor,v2);
                helpMarker("Filter sigma in the color space. A larger value of the parameter means that "
                           "farther colors within the pixel neighborhood (see sigmaSpace) will be mixed "
                           "together, resulting in larger areas of semi-equal color.");

                ImGui::InputInt("sigma space",&v3);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_BilateralSigmaSpace,v3);
                helpMarker("Filter sigma in the coordinate space. A larger value of the parameter means that"
                           "farther pixels will influence each other as long as their colors are close enough (see sigmaColor"
                           "). When diameter>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is"
                           "proportional to sigmaSpace.");
            }

            if (ImGui::Button("Filter##bilateral"))
            {
                auto f = [&]() {
                    ProjectManager::get()->filterDisparityBilateral(m_projectData, {m_currentImage});
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            ImGui::SameLine();
            if (ImGui::Button("ALL##bilateral"))
            {
                auto f = [&]() {
                    ProjectManager::get()->filterDisparityBilateral(m_projectData);
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            ImGui::Separator();

            ImGui::Text("Speckles filter: ");
            {
                auto dispFilterNewValue = imPair ? imPair->denseMatching.dispFilterNewValue : 0;
                auto dispFilterMaxSpeckleSize = imPair ? imPair->denseMatching.dispFilterMaxSpeckleSize : 260;
                auto dispFilterMaxDiff  = imPair ? imPair->denseMatching.dispFilterMaxDiff  : 10;

                ImGui::InputInt("new value",&dispFilterNewValue);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispFilterNewValue,dispFilterNewValue);
                ImGui::InputInt("max speckle size",&dispFilterMaxSpeckleSize);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispFilterMaxSpeckleSize,dispFilterMaxSpeckleSize);
                ImGui::InputInt("max diff",&dispFilterMaxDiff);
                if (ImGui::IsItemEdited())
                    ProjectManager::get()->setProperty(&imPair->denseMatching,p3dDense_DispFilterMaxDiff,dispFilterMaxDiff);
            }

            if (ImGui::Button("Filter##speckles"))
            {
                auto f = [&]() {
                    ProjectManager::get()->filterDisparitySpeckles(m_projectData, {m_currentImage});
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            ImGui::SameLine();
            if (ImGui::Button("ALL##speckles"))
            {
                auto f = [&]() {
                    ProjectManager::get()->filterDisparitySpeckles(m_projectData);
                    m_textureNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }

            if (disableButtons) ImGuiC::PopDisabled();
        }

        ImGui::EndTabItem();

        if (m_currentTab != Tab_Stereo) _resetAppState();
        m_currentTab = Tab_Stereo;
    }
}

void Application::_drawTab_Multiview()
{
    if (ImGui::BeginTabItem(ICON_FA_LAYER_GROUP""))
    {
        if (ImGui::CollapsingHeader("Measurement matrix",m_collapsingHeaderFlags))
        {
            if (ImGui::Button("Get full W"))
            {
                auto f = [&]() {
                    ProjectManager::get()->findMeasurementMatrixFull(m_projectData);
                };
                _doHeavyTask(f);
            }

            if (ImGui::Button("Get W"))
            {
                auto f = [&]() {
                    ProjectManager::get()->findMeasurementMatrix(m_projectData);
                };
                _doHeavyTask(f);
            }
        }

        if (ImGui::CollapsingHeader("Autocalibration",m_collapsingHeaderFlags))
        {
            if (ImGui::Button("Autocalibrate"))
            {
                auto f = [&]() {
                    ProjectManager::get()->autocalibrate(m_projectData);
                    m_viewer3dNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
        }

        if (ImGui::CollapsingHeader("Triangulation",m_collapsingHeaderFlags))
        {
            if (ImGui::Button("Triangulate (sparse)"))
            {
                auto f = [&]() {
                    ProjectManager::get()->triangulate(m_projectData);
                    m_viewer3dNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            if (ImGui::Button("Triangulate dense (stereo)"))
            {
                auto f = [&]() {
                    ProjectManager::get()->triangulateStereo(m_projectData);
                    m_viewer3dNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
            if (ImGui::Button("Triangulate dense (multi-view)"))
            {
                auto f = [&]() {
                    ProjectManager::get()->triangulateDense(m_projectData);
                    m_viewer3dNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
        }

        if (ImGui::CollapsingHeader("Bundle adjustment",m_collapsingHeaderFlags))
        {
            if (ImGui::Button("Bundle adjustment (sparse)"))
            {
                auto f = [&]() {
                    ProjectManager::get()->bundleAdjustment(m_projectData);
                    m_viewer3dNeedsUpdate = true;
                };
                _doHeavyTask(f);
            }
        }

        if (ImGui::CollapsingHeader("Export",m_collapsingHeaderFlags))
        {
            if (ImGui::Button("Export PLY (sparse)"))
            {
                auto f = [&]() {
                    ProjectManager::get()->exportPLY(m_projectData);
                };
                _doHeavyTask(f);
            }
        }

        if (ImGui::CollapsingHeader("Analyze",m_collapsingHeaderFlags))
        {
            static bool showReprojectionErrorPlot = false;
            if (ImGui::Button("Plot reprojection error (sparse)"))
            {
                showReprojectionErrorPlot = true;
            }

            if (showReprojectionErrorPlot) {
                float w = 600;
                ImGui::SetNextWindowSize(ImVec2(w,w));
                if (ImGui::Begin("Reproj error##plot",&showReprojectionErrorPlot,ImGuiWindowFlags_Modal)) {

                    ImVec2 a1 = ImGui::GetWindowPos();
                    ImVec2 center = ImVec2(a1.x+w/2.0f,a1.y+w/2.0f);

                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    ImU32 color32 = IM_COL32(255,255,255,255*0.4f);
                    draw_list->AddLine(ImVec2(center.x,center.y - w*0.5f), ImVec2(center.x,center.y + w*0.5f), color32);
                    draw_list->AddLine(ImVec2(center.x - w*0.5f,center.y), ImVec2(center.x + w*0.5f,center.y), color32);
                    draw_list->AddCircle(center, w*0.5f*0.4f, color32, 32, 1);
                    draw_list->AddCircle(center, w*0.5f*0.8f, color32, 32, 1);

                    auto X = m_projectData.getPts3DSparse();
                    auto W = m_projectData.getMeasurementMatrixFull();
                    auto P = m_projectData.getCameraMatricesMat();

                    Mat error = W - P * X;
                    int less1px = 0;
                    int less05px = 0;
                    int nbCams = P.rows() / 3;
                    for (int c = 0; c < P.rows() / 3; ++c) {
                        ImU32 color2 = IM_COL32(0,255*c/float(nbCams),255,255*0.9f);
                        for (int i = 0; i < error.cols(); ++i) {
                            Vec2 e = error.block(3*c,i,2,1);
                            if (e.norm() < 1.0f) less1px++;
                            if (e.norm() < 0.5f) less05px++;

                            e *= w*0.5f*0.8f;
                            e(0) += center.x;
                            e(1) += center.y;
                            draw_list->AddCircle(ImVec2(e(0),e(1)), 2, color2, 6, 1);
                        }
                    }

                    ImGui::Text("From %i points", int(error.cols()));
                    ImGui::Text("< 1px: %0.1f", 100.0f * less1px / float(nbCams * error.cols()));
                    ImGui::Text("< 0.5px: %0.1f", 100.0f * less05px / float(nbCams * error.cols()));

                }
                ImGui::End();
            }
        }

        ImGui::EndTabItem();

        if (!isOneOf(m_currentTab, {Tab_Multiview,Tab_PointCloud})) _resetAppState();
        m_currentTab = Tab_Multiview;
    }
}

void Application::_drawData()
{
    if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {

        if (m_currentImage >= m_projectData.nbImages()) {
            m_currentImage = 0;
            _resetAppState();
        }

        if (ImGui::TreeNodeEx("Image list:", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (int n = 0; n < m_projectData.nbImages(); n++)
            {
                auto imPtr = m_projectData.image(n);
                if (!imPtr) continue;

                std::string entry = ICON_FA_IMAGE" " + imPtr->name();
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

        if (ImGui::TreeNodeEx("Image pairs:", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (auto n = 0; n < (int)m_projectData.nbImagePairs(); n++)
            {
                auto imL = m_projectData.imagePairL(n);
                auto imR = m_projectData.imagePairR(n);
                if (!imL) continue;
                if (!imR) continue;

                std::string entry = ICON_FA_IMAGES" " + imL->name() + " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n || m_currentSection != Section_Default) {
                        m_currentImage = n;
                        m_currentSection = Section_Default;
                        m_textureNeedsUpdate = true;
                        LOG_DBG("Selection changed: %i", n);
                        LOG_DBG(" - section: %i", m_currentSection);

                    }
                }
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Rectified:", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (auto n = 0; n < (int)m_projectData.nbImagePairs(); n++)
            {
                auto imPair = m_projectData.imagePair(n);
                if (!imPair || !imPair->isRectified()) continue;

                auto imL = m_projectData.imagePairL(n);
                auto imR = m_projectData.imagePairR(n);
                if (!imL) continue;
                if (!imR) continue;

                std::string entry = ICON_FA_ALIGN_CENTER" " + imL->name() + " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n || m_currentSection != Section_Rectified) {
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
        if (ImGui::TreeNodeEx("Disparity:", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (auto n = 0; n < (int)m_projectData.nbImagePairs(); n++)
            {
                auto imPair = m_projectData.imagePair(n);
                if (!imPair || !imPair->hasDisparityMap()) continue;

                auto imL = m_projectData.imagePairL(n);
                auto imR = m_projectData.imagePairR(n);
                if (!imL) continue;
                if (!imR) continue;

                std::string entry = ICON_FA_ALIGN_CENTER" " + imL->name() + " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n || m_currentSection != Section_DisparityMap) {
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
}

void Application::_drawProperties()
{
    ImGui::Columns(2, "mycolumns3", true);  // 2 cols with border

    if (m_currentTab == Tab_General || m_currentTab == Tab_Image)
    {
        auto im = m_projectData.image(m_currentImage);
        if (!im) return;

        drawProperty_basic(im->getPath().c_str(),"path","%s");
        drawProperty_basic(im->getNbFeatures(),"features","%i");
        if (!im->getTranslation().isApprox(Vec2(0,0))){
            drawProperty_matrix(im->getCamera().getK(),"K","Calibration matrix (intrinsic parameters)");
            drawProperty_matrix(im->getTranslation(),"t","Camera translation (extrinsic parameters)");
        }
        return;
    }

    if (m_currentTab == Tab_Stereo)
    {
        auto imPair = m_projectData.imagePair(m_currentImage);
        if (!imPair) return;

        if (imPair->hasMatches())
            drawProperty_basic(imPair->getNbMatches(),"matches","%i");

        if (imPair->hasF()) {
            drawProperty_matrix(imPair->getFundMat(),"F","Fundamental matrix");
            drawProperty_basic(utils::rad2deg(imPair->getTheta1()),"theta","%0.3f deg");
            drawProperty_basic(utils::rad2deg(imPair->getTheta2()),"theta'","%0.3f deg");
        }

        if (imPair->hasRrelative()) {
            drawProperty_basic(utils::rad2deg(imPair->getRho()),"rho","%0.3f deg");
            drawProperty_matrix(imPair->getRrelative(),"R","Relative rotation between images");
        }
        return;
    }

    if (m_currentTab == Tab_Multiview)
    {
        const auto & Wfull = m_projectData.getMeasurementMatrixFull();
        if (Wfull.rows() > 0 && Wfull.cols() > 0)
            drawProperty_matrix(Wfull,"Wf","Full measurement matrix");

        const auto & W = m_projectData.getMeasurementMatrix();
        if (W.rows() > 0 && W.cols() > 0)
            drawProperty_matrix(W,"W","Measurement matrix");

        const auto & P = m_projectData.getCameraMatricesMat();
        if (P.rows() > 0 && P.cols() > 0)
            drawProperty_matrix(P,"P","Concatenated camera matrices");

        const auto & Xf = m_projectData.getPts3DSparse();
        if (Xf.cols() > 0)
            drawProperty_matrix(Xf,"Xf","3D points from Wf");

        const auto & Xd = m_projectData.getPts3DDense();
        if (Xd.cols() > 0)
            drawProperty_matrix(Xd,"Xd","Dense point cloud");

        return;
    }

    ImGui::Columns(1);
}


template<typename Type>
void Application::drawProperty_basic(const Type &v, const std::string &name, const char *fmt, const char *icon)
{
    bool hovered = false;
    auto icone = (icon != nullptr) ? icon : ICON_FA_CUBE;
    ImGui::Text("%s %s", icone, name.c_str());
    if (ImGui::IsItemHovered()) hovered = true;
    ImGui::NextColumn();
    ImGui::Text(fmt,v);
    if (hovered || ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text(fmt,v);
        ImGui::EndTooltip();
    }
    ImGui::NextColumn();
}

template<typename Scalar, int SizeX, int SizeY>
void Application::drawProperty_matrix(
        const Eigen::Matrix<Scalar, SizeX, SizeY> &A,
        const std::string &name, const std::string &longName)
{
    static Eigen::IOFormat CleanFmt(5, 0, " ", "##", "", "");
    static Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    static Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "];");

    int rows = (SizeX == -1) ? static_cast<int>(A.rows()) : SizeX;
    int cols = (SizeY == -1) ? static_cast<int>(A.cols()) : SizeY;

    bool hovered = false;
    ImGui::Text("%s %s", ICON_FA_BORDER_ALL, name.c_str());
    std::string popupName = "##matrix_prop_popup" + name + longName;
    if (ImGui::IsItemHovered()) hovered = true;
    ImGui::OpenPopupOnItemClick(popupName.c_str(), 1);

    ImGui::NextColumn();
    ImGui::Text("[%ix%i]", rows, cols);
    if (hovered || ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();

        if (longName != "")
            ImGui::Text("%s\n", longName.c_str());

        int maxX = 12;
        int maxY = 12;
        if (rows > maxX || cols > maxY) {
            ImGui::PushStyleColor(ImGuiCol_Text, COLOR_WARN);
            ImGui::Text(ICON_FA_EXCLAMATION_TRIANGLE" Matrix is too big to display. Showing only the %ix%i top left corner\n\n", maxX, maxY);
            ImGui::PopStyleColor();

        }
        ImGui::PushFont(m_fontMono);

        maxX = std::min(maxX,rows);
        maxY = std::min(maxY,cols);
        const auto & block = A.block(0,0,maxX,maxY);
        std::stringstream ss;
        ss << block.format(CleanFmt);
        std::string output = ss.str();

        auto rows = utils::split(output,"##");
        for (const auto & row : rows){
            ImGui::Text("%s",row.c_str());
        }

        ImGui::PopFont();
        ImGui::EndTooltip();
    }
    if (ImGui::BeginPopupContextItem(popupName.c_str()))
    {
        if(ImGui::Selectable("Copy to clipboard: Eigen")) {
            std::stringstream ss;
            ss << A.format(CommaInitFmt);
            std::string output = ss.str();

            ImGui::LogToClipboard();
            ImGui::LogText("%s.setZero(%li;%li);\nM %s", name.c_str(), A.rows(), A.cols(), output.c_str());
            LOG_OK("Matrix %s is copied to clipboard", name.c_str());
            ImGui::CloseCurrentPopup();
        }
        if(ImGui::Selectable("Copy to clipboard: Octave/MATLAB")) {
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
    float width  = ImGui::GetWindowContentRegionMax().x;
    float height = ImGui::GetWindowContentRegionMax().y;

    if (m_currentTab == Tab_Multiview && m_viewer3D != nullptr) {
        m_viewer3D->setSize(width,height);

        if (m_viewer3dNeedsUpdate) {
            m_viewer3D->init();
            m_viewer3D->setPointCloud(
                        m_projectData.getPts3DDense(),
                        m_projectData.getPts3DDenseColors()
                        );
            m_viewer3dNeedsUpdate = false;
        }

        m_viewer3D->draw();
        m_textureWidth = width;
        m_textureHeight = height;
        setTextureId(m_viewer3D->textureId());
        textureDisplay(ImVec2(width, height),ImVec2(0,0),ImVec2(1,1));
        return;
    }

    if (isOneOf(m_currentTab,{Tab_General,Tab_Image,Tab_Stereo})) {
        const int SHOW_MATCHES = 0;
        const int SHOW_EPILINES = 1;
        static int display = SHOW_MATCHES;

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
                    if (m_currentSection == Section_Default) {
                        auto imIdxL = imPair->imL();
                        auto imIdxR = imPair->imR();
                        auto imL = m_projectData.image(imIdxL);
                        auto imR = m_projectData.image(imIdxR);

                        if (imL && imR)
                            matToBind = utils::concatenateCvMat({imL->cvMat(),imR->cvMat()},utils::CONCAT_HORIZONTAL);
                    } else if (m_currentSection == Section_Rectified) {
                        const auto & imL = imPair->getRectifiedImageL();
                        const auto & imR = imPair->getRectifiedImageR();
                        if (!imL.empty() && !imR.empty())
                            matToBind = utils::concatenateCvMat({imL,imR},utils::CONCAT_HORIZONTAL);
                    } else if (m_currentSection == Section_DisparityMap) {
                        matToBind = imPair->getDisparityMapPlot();
                    }
                }
            }

            if (!matToBind.empty()) textureBind(matToBind);
            else {
                m_textureHeight = 0;
                m_textureWidth  = 0;
            }
            m_textureNeedsUpdate = false;
        }

        if (isTextureReady()) {
            float width  = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - controlBottomBarHeight;
            float texAspect = m_textureHeight / float(m_textureWidth);
            float winAspect = height / float(width);

            float newW, newH;

            float imStartX = 0;
            float imStartY = 0;
            if (texAspect > winAspect){
                newH = height;
                newW = newH / texAspect;
                imStartX = (width - newW)/2.0f;
            } else {
                newW = width;
                newH = newW * texAspect;
                imStartY = (height - newH)/2.0f;
            }

            ImGui::SetCursorPos(ImVec2(imStartX,imStartY));
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, imageOpacity);
            textureDisplay(ImVec2(newW, newH),ImVec2(0,0),ImVec2(1,1));
            ImGui::PopStyleVar();

            ImVec2 winPos = ImGui::GetWindowPos();
            const float posX = winPos.x + imStartX;
            const float posY = winPos.y + imStartY;

            if (isOneOf(m_currentTab,{Tab_General,Tab_Image})) {
                if (showFeatures) _showFeatures(ImVec2(posX, posY), ImVec2(newW, newH),color,featuresSize);
            } else if (m_currentTab == Tab_Stereo) {
                if (m_currentSection == Section_Default) {
                    if (display == SHOW_MATCHES) _showMatches(ImVec2(posX, posY), ImVec2(newW, newH),color,lineWidth,skipEvery);
                    else if (display == SHOW_EPILINES) _showEpilines(ImVec2(posX, posY), ImVec2(newW, newH),color,lineWidth,skipEvery);
                } else if (m_currentSection == Section_Rectified) {
                    // just draw a horizontal line to judge the quality of rectification
                    if (ImGui::IsItemHovered()) {
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        auto y = ImGui::GetIO().MousePos.y;
                        draw_list->AddLine(ImVec2(posX, y),ImVec2(posX+newW, y), ImColor(200,0,0,120),2);
                    }
                }
            }
        } else {
            ImGui::Text("No image to display");
        }

        ImGui::PushFont(m_fontMonoSmall);
        ImGui::SetCursorPos(ImVec2(0,ImGui::GetWindowHeight() - controlBottomBarHeight));
        ImGui::Separator();
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("##image-opacity", &imageOpacity, 0.0f, 1.0f, "opacity:%.2f");
        ImGui::SameLine();
        if (isOneOf(m_currentTab,{Tab_General,Tab_Image})){
            ImGui::Text("--");
            ImGui::SameLine();
            ImGui::Checkbox("Show features",&showFeatures);
            ImGui::SameLine();
            ImGui::Text("--");
            ImGui::SameLine();
            ImGui::SliderFloat("##feature-size",&featuresSize, 1.0f,15.0f, "feat_size:%.1f");
        } else if (m_currentTab == Tab_Stereo) {
            if (m_currentSection == Section_Default) {
                ImGui::Text("--"); ImGui::SameLine();
                ImGui::RadioButton("matches", &display, 0); ImGui::SameLine();
                ImGui::RadioButton("epilines", &display, 1);
            }
            ImGui::SameLine();
            const char * sliderText = (skipEvery == 1) ? "show all" : "skip every:%i";
            ImGui::SliderInt("##percentage_of_features", &skipEvery, 1, 10, sliderText);
            ImGui::SameLine();
            ImGui::SliderFloat("##lineWidth", &lineWidth, 1.0f, 5.0f, "line width");
        }
        if (isOneOf(m_currentTab,{Tab_General,Tab_Image})/* || (display == SHOW_MATCHES)*/) {
            ImGui::SameLine();
            ImGui::ColorEdit4("MyColor##3", (float*)&color, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel );
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

    //ImGui::CaptureKeyboardFromApp(true);
    ImGuiIO& io = ImGui::GetIO();
    if (io.KeyCtrl) {
        if (ImGui::IsKeyPressed('s') || ImGui::IsKeyPressed('S')) {
            auto f = [&]() {
                ProjectManager::get()->saveProject(&m_projectData,m_projectData.getProjectPath());
                _resetAppState();
            };
            _doHeavyTask(f);
        } else if (ImGui::IsKeyPressed(ImGuiKey_Z)){
            m_textureNeedsUpdate = true;
            CommandManager::get()->undoCommand();
        } else if (ImGui::IsKeyPressed(ImGuiKey_Tab)) {

        }
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
        LOG_DBG("Up pressed");
        if (!isOneOf(m_currentTab,{Tab_General,Tab_Image,Tab_Stereo})) return;

        int modulo = m_projectData.nbImages();
        if (m_currentTab == Tab_Stereo) modulo = m_projectData.nbImagePairs();
        if (modulo > 0) {
            m_currentImage = (m_currentImage - 1 + modulo) % modulo;
            m_textureNeedsUpdate = true;
        }

    } else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
        LOG_DBG("Down pressed");

        if (!isOneOf(m_currentTab,{Tab_General,Tab_Image,Tab_Stereo})) return;

        int modulo = m_projectData.nbImages();
        if (m_currentTab == Tab_Stereo) modulo = m_projectData.nbImagePairs();

        if (modulo > 0) {
            m_currentImage = (m_currentImage + 1) % modulo;
            m_textureNeedsUpdate = true;
        }
    }
}

void Application::_showFeatures(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float featuresSize)
{
    auto im = m_projectData.image(m_currentImage);
    if (im && im->hasFeatures()){
        const auto & keyPts = im->getKeyPoints();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        const auto color32 = IM_COL32(col.x*255,col.y*255,col.z*255,col.w*255);

        for (const auto & kpt : keyPts) {
            float x = pos.x + kpt.pt.x / float(m_textureWidth) * size.x;
            float y = pos.y + kpt.pt.y / float(m_textureHeight) * size.y;


            draw_list->AddCircle(ImVec2(x, y), featuresSize, color32, 6, 2);
        }
    }
}

void Application::_showEpilines(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float lineWidth, int skipEvery)
{
    std::vector<Vec2> ptsL, ptsR;
    m_projectData.getPairwiseMatches(m_currentImage,ptsL,ptsR);

    auto imPair = m_projectData.imagePair(m_currentImage);
    if (ptsL.empty() || ptsR.empty()) return;

    auto imL = m_projectData.image(imPair->imL());
    auto imR = m_projectData.image(imPair->imR());

    if (!imL || !imR) return;

    // the texture is rendered started in {pos} with the size {size}
    float ratioLeftTotal = imL->width() / float(imL->width() + imR->width());
    float ratioRightTotal = (1.0f - ratioLeftTotal);

    if (imL->height() != imR->height()) {
        LOG_ERR("Showing epilines is not supported yet for images with different height");
    }

    const float posXr = ratioLeftTotal * size.x;

    std::vector<Vec3> epilinesL,epilinesR;
    imPair->getEpilinesLeft(ptsR,epilinesL);
    imPair->getEpilinesRight(ptsL,epilinesR);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    auto mapToTextureL = [&] ( const Vec2 & pt) -> ImVec2 {
        return ImVec2(pos.x + float(pt[0]) / float(imL->width()) * ratioLeftTotal * size.x,
                      pos.y + float(pt[1]) / float(imL->height()) * size.y);
    };

    auto mapToTextureR = [&] ( const Vec2 & pt) -> ImVec2 {
        return ImVec2(pos.x + posXr + float(pt[0]) / float(imR->width()) * ratioRightTotal * size.x,
                      pos.y + float(pt[1]) / float(imR->height()) * size.y);
    };

    static int hoveredLine = -1;
    bool wasHoveredThisFrame = false;
    float radius = 1.5f + lineWidth * 1.5f;
    for (int i = 0; i < ptsL.size(); i += skipEvery) {
        const auto & ptL = ptsL[i];
        const auto & ptR = ptsR[i];

        auto epilinePtsL = utils::lineIntersectBox(epilinesL[i],imL->width(),imL->height());
        auto epilinePtsR = utils::lineIntersectBox(epilinesR[i],imR->width(),imR->height());

        ImU32 color32 = IM_COL32(125,125,125,125);
        if (hoveredLine == -1) {
            Vec3f color = palette::color1(i / float(ptsL.size()));
            color32 = IM_COL32(color(0)*255,color(1)*255,color(2)*255,255);
        }

        ImVec2 leftPoint = mapToTextureL(ptL);
        ImVec2 rightPoint = mapToTextureR(ptR);

        std::string featureLblL = "##featureL-" + std::to_string(i);
        std::string featureLblR = "##featureR-" + std::to_string(i);
        ImGuiC::ItemCircle(featureLblL.c_str(),leftPoint,radius,color32);
        if (ImGui::IsItemHovered()) { hoveredLine = i; wasHoveredThisFrame = true;}
        ImGuiC::ItemCircle(featureLblR.c_str(),rightPoint,radius,color32);
        if (ImGui::IsItemHovered()) { hoveredLine = i; wasHoveredThisFrame = true;}

//        draw_list->AddCircle(leftPoint, 2, color32, 6, 3);
//        draw_list->AddCircle(rightPoint, 2, color32, 6, 3);
        draw_list->AddLine(mapToTextureL(epilinePtsL.first),mapToTextureL(epilinePtsL.second), color32,lineWidth);
        draw_list->AddLine(mapToTextureR(epilinePtsR.first),mapToTextureR(epilinePtsR.second), color32,lineWidth);
    }

    if (hoveredLine != -1) {
        const auto & ptL = ptsL[hoveredLine];
        const auto & ptR = ptsR[hoveredLine];
        auto epilinePtsL = utils::lineIntersectBox(epilinesL[hoveredLine],imL->width(),imL->height());
        auto epilinePtsR = utils::lineIntersectBox(epilinesR[hoveredLine],imR->width(),imR->height());

        Vec3f color = palette::color1(hoveredLine / float(ptsL.size()));
        ImU32 color32 = IM_COL32(color(0)*255,color(1)*255,color(2)*255,255);
        draw_list->AddCircle( mapToTextureL(ptL), radius, color32, 6, 2);
        draw_list->AddCircle( mapToTextureR(ptR), radius, color32, 6, 2);
        draw_list->AddLine(mapToTextureL(epilinePtsL.first),mapToTextureL(epilinePtsL.second), color32,lineWidth);
        draw_list->AddLine(mapToTextureR(epilinePtsR.first),mapToTextureR(epilinePtsR.second), color32,lineWidth);
    }

    if (!wasHoveredThisFrame) hoveredLine = -1;
}

void Application::_showMatches(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float lineWidth, int skipEvery)
{
    std::vector<Vec2> ptsL, ptsR;
    m_projectData.getPairwiseMatches(m_currentImage,ptsL,ptsR);

    auto imPair = m_projectData.imagePair(m_currentImage);
    if (ptsL.empty() || ptsR.empty()) return;

    auto imL = m_projectData.image(imPair->imL());
    auto imR = m_projectData.image(imPair->imR());

    if (!imL || !imR) return;

    // the texture is rendered started in {pos} with the size {size}
    float ratioLeftTotal = imL->width() / float(imL->width() + imR->width());
    float ratioRightTotal = (1.0f - ratioLeftTotal);

    if (imL->height() != imR->height()) {
        LOG_ERR("Showing epilines is not supported yet for images with different height");
    }

    const float posXr = ratioLeftTotal * size.x;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    auto mapToTextureL = [&] ( const Vec2 & pt) -> ImVec2 {
        return ImVec2(pos.x + float(pt[0]) / float(imL->width()) * ratioLeftTotal * size.x,
                      pos.y + float(pt[1]) / float(imL->height()) * size.y);
    };

    auto mapToTextureR = [&] ( const Vec2 & pt) -> ImVec2 {
        return ImVec2(pos.x + posXr + float(pt[0]) / float(imR->width()) * ratioRightTotal * size.x,
                      pos.y + float(pt[1]) / float(imR->height()) * size.y);
    };

    static int hoveredLine = -1;
    bool wasHoveredThisFrame = false;
    float radius = 1.5f + lineWidth * 1.5f;
    for (int i = 0; i < ptsL.size(); i += skipEvery) {
        const auto & ptL = ptsL[i];
        const auto & ptR = ptsR[i];

        ImU32 color32 = IM_COL32(125,125,125,125);
        if (hoveredLine == -1) {
            Vec3f color = palette::color1(i / float(ptsL.size()));
            color32 = IM_COL32(color(0)*255,color(1)*255,color(2)*255,255);
        }

        ImVec2 leftPoint = mapToTextureL(ptL);
        ImVec2 rightPoint = mapToTextureR(ptR);

        std::string featureLblL = "##featureL-" + std::to_string(i);
        std::string featureLblR = "##featureR-" + std::to_string(i);
        ImGuiC::ItemCircle(featureLblL.c_str(),leftPoint,radius,color32);
        if (ImGui::IsItemHovered()) { hoveredLine = i; wasHoveredThisFrame = true;}
        ImGuiC::ItemCircle(featureLblR.c_str(),rightPoint,radius,color32);
        if (ImGui::IsItemHovered()) { hoveredLine = i; wasHoveredThisFrame = true;}

        //draw_list->AddCircle( leftPoint, 2, color32, 6, 1 + lineWidth * 1.5f);
        //draw_list->AddCircle(rightPoint, 2, color32, 6, 1 + lineWidth * 1.5f);
        draw_list->AddLine(mapToTextureL(ptL),mapToTextureR(ptR), color32,lineWidth);
    }

    if (hoveredLine != -1) {
        const auto & ptL = ptsL[hoveredLine];
        const auto & ptR = ptsR[hoveredLine];
        Vec3f color = palette::color1(hoveredLine / float(ptsL.size()));
        ImU32 color32 = IM_COL32(color(0)*255,color(1)*255,color(2)*255,255);
        draw_list->AddCircle( mapToTextureL(ptL), radius, color32, 6, 2);
        draw_list->AddCircle( mapToTextureR(ptR), radius, color32, 6, 2);
        draw_list->AddLine(mapToTextureL(ptL),mapToTextureR(ptR), color32,lineWidth);
    }

    if (!wasHoveredThisFrame) hoveredLine = -1;
}
