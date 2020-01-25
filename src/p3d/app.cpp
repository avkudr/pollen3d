#include "app.h"

#include <future>

#include "p3d/project_manager.h"
#include "p3d/commands.h"

#ifndef P3D_PROJECT_EXTENSION
#define P3D_PROJECT_EXTENSION ".yml.gz"
#endif

void Application::initImGui(){
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.Fonts->AddFontFromFileTTF("../assets/Ubuntu-R.ttf", 16.0f);
    m_fontMonoSmall = io.Fonts->AddFontFromFileTTF("../assets/UbuntuMono-R.ttf", 16.0f);
    ConsoleLogger::get()->setFont(m_fontMonoSmall);
    m_fontMonoSmall = io.Fonts->AddFontFromFileTTF("../assets/UbuntuMono-R.ttf", 14.0f);
}

void Application::applyStyle() {
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.FrameRounding = 6;
    style.IndentSpacing = 12.0f;
}

void Application::update(int width, int height){
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, m_heightTabSection),ImVec2(width, m_heightTabSection));
    _renderTabWidget();

    static bool firstCall = true;
    ImGui::SetNextWindowBgAlpha(0.5);
    ImGui::SetNextWindowPos(ImVec2(0, m_heightTabSection),ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, height - m_heightTabSection),ImGuiCond_Always);
    if (ImGui::Begin("Central area",nullptr,ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings)) {
    //    ImGuiID dockspaceID = ImGui::GetID("ID().c_str()");
    //    ImGui::DockSpace(dockspaceID);

        ImGui::BeginColumns("columns", 3);//false);
        if (firstCall) {
            firstCall = false;
            ImGui::SetColumnWidth(0,width * 0.2f);
            ImGui::SetColumnWidth(1,width * 0.5f);
            ImGui::SetColumnWidth(2,width * 0.3f);
        }

        _renderLeftWidget();

        ImGui::NextColumn();

        _renderCentralWidget();

        ImGui::NextColumn();
        ConsoleLogger::get()->render();
        ImGui::NextColumn();
        ImGui::EndColumns();
        ImGui::End();
    }

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

void Application::_renderTabWidget()
{

    static bool showImGuiMetrics = false;
    static bool showDemoImGui = false;

    ImGui::Begin("##tab-widget",nullptr,
                 ImGuiWindowFlags_NoDocking |
                 ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_MenuBar);

    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Menu"))
        {
            ImGui::MenuItem("Main menu bar");
            ImGui::MenuItem("Console");
            ImGui::MenuItem("Log");
            ImGui::MenuItem("Simple layout");
            ImGui::MenuItem("Property editor");
            ImGui::MenuItem("Long text display");
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

    if (showImGuiMetrics) ImGui::ShowMetricsWindow();
    if (showDemoImGui) ImGui::ShowDemoWindow();

    if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_TabListPopupButton))
    {
        _renderTab_General();
        _renderTab_Image();
        _renderTab_Stereo();

        if (ImGui::BeginTabItem("Multiview"))
        {
            ImGui::Text("ID: 0123456789");
            ImGui::EndTabItem();
            if (!isOneOf(m_currentTab, {Tab_Multiview,Tab_PointCloud})) m_textureNeedsUpdate = true;
            m_currentTab = Tab_Multiview;
        }
        if (ImGui::BeginTabItem("Point cloud"))
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1,0,0,1));
            ImGui::Text("ID: 0123456789");
            ImGui::PopStyleColor();
            ImGui::EndTabItem();
            if (!isOneOf(m_currentTab, {Tab_Multiview,Tab_PointCloud})) m_textureNeedsUpdate = true;
            m_currentTab = Tab_PointCloud;
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void Application::_renderTab_General()
{
    if (!ImGui::BeginTabItem("General")) return;

    int buttonH = 100;
    if (ImGui::Button("Load\nimages",ImVec2(70, buttonH)))
    {
        auto files = ProjectManager::get()->loadImagesDialog();
        auto f = [&,files]() {
            ProjectManager::get()->loadImages(&m_projectData,files);
            if (!m_projectData.empty()) m_currentImage = 0;
            m_textureNeedsUpdate = true;
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save\nproject",ImVec2(70, buttonH)))
    {
        auto f = [&]() {
            ProjectManager::get()->saveProject(&m_projectData,m_projectData.getProjectPath());
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save\nproject\nas...",ImVec2(70, buttonH)))
    {
        auto file = ProjectManager::get()->saveProjectDialog();
        ProjectManager::get()->saveProject(&m_projectData, file);
    }
    ImGui::SameLine();
    if (ImGui::Button("Open\nproject",ImVec2(70, buttonH)))
    {
        auto file = ProjectManager::get()->openProjectDialog();
        LOG_DBG("Open project: %s", file.c_str());

        auto f = [&,file]() {
            ProjectManager::get()->openProject(&m_projectData, file);
            m_currentImage = 0;
            m_textureNeedsUpdate = true;
        };
        if (file != "") _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG"))
    {
        ProjectManager::get()->extractFeatures(m_projectData, {m_currentImage});
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_LOAD"))
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
            m_textureNeedsUpdate = true;
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_SAVE"))
    {
        ProjectManager::get()->saveProject(&m_projectData,"/home/andrey/Projects/pollen3d/build/test_project" + std::string(P3D_PROJECT_EXTENSION));
    }
    ImGui::SameLine();
    if (ImGui::Button("DBG_PROJ"))
    {
        auto f = [&]() {
            ProjectManager::get()->openProject(&m_projectData, "/home/andrey/Projects/pollen3d/build/test_project" + std::string(P3D_PROJECT_EXTENSION));
            m_currentImage = 0;
            m_textureNeedsUpdate = true;
        };
        _doHeavyTask(f);
    }
    ImGui::SameLine();
    ImGui::EndTabItem();

    if (!isOneOf(m_currentTab, {Tab_General,Tab_Image})) m_textureNeedsUpdate = true;
    m_currentTab = Tab_General;
}

void Application::_renderTab_Image()
{
    if (ImGui::BeginTabItem("Image"))
    {
        ImGui::BeginColumns("columns", 1);//false);
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
        //ImGui::NextColumn();



        //ImGui::NextColumn();

        ImGui::NextColumn();
        ImGui::EndColumns();

        ImGui::EndTabItem();

        if (!isOneOf(m_currentTab, {Tab_General,Tab_Image})) m_textureNeedsUpdate = true;
        m_currentTab = Tab_Image;
    }
}

void Application::_renderTab_Stereo()
{
    if (ImGui::BeginTabItem("Stereo"))
    {
        int matchingWidgetW = 250;

        if (ImGui::BeginChild("Matching",ImVec2(matchingWidgetW,ImGui::GetContentRegionAvail().y)))
        {
            ImGui::BulletText("Matching");
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

            if (ImGui::Button("Match features",ImVec2(0.6*matchingWidgetW,50)))
            {
                auto f = [&]() {
                    ProjectManager::get()->matchFeatures(m_projectData, {m_currentImage});
                };
                _doHeavyTask(f);
            }

            ImGui::SameLine();
            if (ImGui::Button("ALL",ImVec2(0.25*matchingWidgetW,50)))
            {
                auto f = [&]() {
                    ProjectManager::get()->matchFeatures(m_projectData);
                };
                _doHeavyTask(f);
            }
            ImGui::EndChild();
        }

        ImGui::SameLine();

        if (ImGui::BeginChild("EpipolarGeometry",ImVec2(matchingWidgetW,ImGui::GetContentRegionAvail().y)))
        {
            ImGui::BulletText("Epipolar geometry");

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

            ImGui::SetNextItemWidth(matchingWidgetW);
            if (ImGui::Button("Match features"))
            {
                auto f = [&]() {
                    ProjectManager::get()->matchFeatures(m_projectData, {m_currentImage});
                };
                _doHeavyTask(f);
            }

            ImGui::SetNextItemWidth(matchingWidgetW);
            if (ImGui::Button("ALL"))
            {
                auto f = [&]() {
                    ProjectManager::get()->matchFeatures(m_projectData);
                };
                _doHeavyTask(f);
            }
            ImGui::EndChild();
        }

        ImGui::EndTabItem();

        if (m_currentTab != Tab_Stereo) m_textureNeedsUpdate = true;
        m_currentTab = Tab_Stereo;
    }
}

void Application::_renderLeftWidget()
{
    if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {

        if (m_currentImage >= m_projectData.nbImages()) {
            m_currentImage = 0;
            m_textureNeedsUpdate = true;
        }

        if (ImGui::TreeNodeEx("Image list:", ImGuiTreeNodeFlags_DefaultOpen))
        {
            for (int n = 0; n < m_projectData.nbImages(); n++)
            {
                auto imPtr = m_projectData[n];
                if (!imPtr) continue;

                if (ImGui::Selectable(imPtr->name().c_str(), m_currentImage == n)) {
                    if (m_currentImage != n) {
                        m_currentImage = n;
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
                auto imPairPtr = m_projectData.imagePair(n);
                if (!imPairPtr) continue;

                auto imL = imPairPtr->imL();
                auto imR = imPairPtr->imR();
                if (!imL) continue;
                if (!imR) continue;

                std::string entry = imL->name() + " <> " + imR->name();
                if (ImGui::Selectable(entry.c_str(), m_currentImage == n)) {
                    if (m_currentImage != n) {
                        m_currentImage = n;
                        m_textureNeedsUpdate = true;
                        LOG_DBG("Selection changed: %i", n);
                    }
                }
            }
            ImGui::TreePop();
        }
        return;
    }
}

void Application::_renderCentralWidget()
{
    if (!isOneOf(m_currentTab,{Tab_General,Tab_Image,Tab_Stereo})) {
        float width  = ImGui::GetWindowWidth();
        float height = ImGui::GetWindowHeight();
        ImGui::BeginChild("Dummy",ImVec2(width,height));
        ImGui::SetCursorPos(ImVec2(width/2.0f,height/2.0f));
        ImGui::Text("Not implemented yet");
        ImGui::EndChild();
        return;
    }

    static bool showFeatures = true;
    int controlBottomBarHeight = 25;
    static float imageOpacity = 1.0f;
    static float featuresSize = 2.5f;
    static ImVec4 col = COLOR_GREEN;


    if (m_textureNeedsUpdate) {
        cv::Mat matToBind;
        if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {
            auto imPtr = m_projectData.image(m_currentImage);
            if (imPtr) matToBind = imPtr->cvMat();
        } else if (m_currentTab == Tab_Stereo) {
            auto imPairPtr = m_projectData.imagePair(m_currentImage);
            if (imPairPtr) {
                auto imL = imPairPtr->imL();
                auto imR = imPairPtr->imR();
                if (imL && imR) {
                    matToBind = utils::concatenateMat({imL->cvMat(),imR->cvMat()},utils::CONCAT_HORIZONTAL);
                }
            }
        }

        if (!matToBind.empty()) textureBind(matToBind);
        m_textureNeedsUpdate = false;
    }

    ImGui::BeginChild("OpenGL Texture Text");
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
        textureDisplay(newW, newH);
        ImGui::PopStyleVar();

        if (showFeatures && isOneOf(m_currentTab,{Tab_General,Tab_Image})) {
            auto im = m_projectData[m_currentImage];
            if (im && im->isFeaturesExtracted()){
                const auto & keyPts = im->getKeyPoints();
                ImVec2 winPos = ImGui::GetWindowPos();
                ImDrawList* draw_list = ImGui::GetWindowDrawList();

                for (const auto & kpt : keyPts) {
                    float x = kpt.pt.x / m_textureWidth * newW;
                    float y = kpt.pt.y / m_textureHeight * newH;

                    draw_list->AddCircle(ImVec2(winPos.x + imStartX+x, winPos.y + imStartY+y), featuresSize,
                                         IM_COL32(col.x*255,col.y*255,col.z*255,col.w*255), 6, 2);
                }
            }

            ImGui::SetCursorPosY(0);
            ImGui::Text("Size (px): %d x %d", m_textureWidth, m_textureHeight);
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
        ImGui::SameLine();
        ImGui::ColorEdit4("MyColor##3", (float*)&col, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel );
        ImGui::SameLine();
    } else if (m_currentTab == Tab_Stereo) {

    }

    ImGui::PopFont();

    ImGui::EndChild();

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
            };
            _doHeavyTask(f);
        } else if (ImGui::IsKeyPressed(ImGuiKey_Z)){
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
