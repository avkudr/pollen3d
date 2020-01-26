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
    io.ConfigDockingWithShift = true;

    io.Fonts->AddFontFromFileTTF("../assets/Ubuntu-R.ttf", 16.0f);
    m_fontMonoSmall = io.Fonts->AddFontFromFileTTF("../assets/UbuntuMono-R.ttf", 16.0f);
    ConsoleLogger::get()->setFont(m_fontMonoSmall);
    m_fontMonoSmall = io.Fonts->AddFontFromFileTTF("../assets/UbuntuMono-R.ttf", 14.0f);

    applyStyle();
}

void Application::applyStyle() {
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.FrameRounding  = 4;
    style.WindowRounding = 0;
    style.IndentSpacing = 8.0f;
    style.WindowMenuButtonPosition = ImGuiDir_Right;
}

void Application::draw(int width, int height){
    ImGui::NewFrame();
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, m_heightTabSection),ImVec2(width, m_heightTabSection));
    _drawTab();

    ImGui::SetNextWindowPos(ImVec2(0, m_heightTabSection));
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, height - m_heightTabSection),ImVec2(width, height - m_heightTabSection));
    ImGuiID dock_id = ImGui::GetID("ID().c_str()");
    if (ImGui::Begin("CentralArea",nullptr,ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::DockSpace(dock_id);
        ImGui::End();
    }

    ImGui::Begin("DataWidget",nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDecoration);
    _drawLeft();
    ImGui::End();
    ImGui::Begin("CentralWidget",nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDecoration);
    _drawCentral();
    ImGui::End();
    ImGui::Begin("Console",nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDecoration);
    ConsoleLogger::get()->render();
    ImGui::End();

    ImGui::PopStyleVar();

    if (m_dockingNeedsReset){
        ImGuiID dock_id = ImGui::GetID("ID().c_str()");

        ImGui::DockBuilderRemoveNode(dock_id); // Clear out existing layout
        ImGui::DockBuilderAddNode(dock_id); //viewport->Size); // Add empty node

        ImGuiID id1, id2;
        ImGuiID id3, id4;
        ImGui::DockBuilderSetNodePos(dock_id, ImVec2(0, m_heightTabSection));
        ImGui::DockBuilderSetNodeSize(dock_id, ImVec2(width, height - m_heightTabSection));
        ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Right,0.35f,&id2,&id1);
        ImGui::DockBuilderSplitNode(id1, ImGuiDir_Left,0.25f,&id3,&id4);
        ImGui::DockBuilderDockWindow("Console", id2);
        ImGui::DockBuilderDockWindow("DataWidget", id3);
        ImGui::DockBuilderDockWindow("CentralWidget", id4);

        ImGui::DockBuilderGetNode(id2)->HasCloseButton = false;
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

void Application::_drawTab()
{

    static bool showImGuiMetrics = false;
    static bool showDemoImGui = false;

    ImGui::Begin("##tab-widget",nullptr,
                 ImGuiWindowFlags_NoDocking |
                 ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoTitleBar |
                 ImGuiWindowFlags_MenuBar);

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

    if (showImGuiMetrics) ImGui::ShowMetricsWindow();
    if (showDemoImGui) ImGui::ShowDemoWindow();

    if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_TabListPopupButton))
    {
        _drawTab_General();
        _drawTab_Image();
        _drawTab_Stereo();

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

void Application::_drawTab_General()
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

void Application::_drawTab_Image()
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

void Application::_drawTab_Stereo()
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

void Application::_drawLeft()
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
                auto imPtr = m_projectData.image(n);
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
                if (!imPairPtr || !imPairPtr->isValid()) continue;

                auto imIdxL = imPairPtr->imL();
                auto imIdxR = imPairPtr->imR();
                auto imL = m_projectData.image(imIdxL);
                auto imR = m_projectData.image(imIdxR);
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

void Application::_drawCentral()
{
    if (!isOneOf(m_currentTab,{Tab_General,Tab_Image,Tab_Stereo})) {
        float width  = ImGui::GetWindowContentRegionMax().x;
        float height = ImGui::GetWindowContentRegionMax().y;
        ImGui::BeginChild("Dummy",ImVec2(width,height));
        ImGui::SetCursorPos(ImVec2(width/2.0f,height/2.0f));
        ImGui::Text("Not implemented yet");
        ImGui::EndChild();
        return;
    }

    static bool showFeatures = true;
    static bool showMatches = true;
    int controlBottomBarHeight = 35;
    static float imageOpacity = 1.0f;
    static float featuresSize = 2.5f;
    static ImVec4 col = COLOR_GREEN;

    if (m_textureNeedsUpdate) {
        cv::Mat matToBind;
        if (m_currentTab == Tab_General || m_currentTab == Tab_Image) {
            auto imPtr = m_projectData.image(m_currentImage);
            if (imPtr) matToBind = imPtr->cvMat();
        } else if (m_currentTab == Tab_Stereo) {
            auto imPair = m_projectData.imagePair(m_currentImage);
            if (imPair && imPair->isValid()) {
                auto imIdxL = imPair->imL();
                auto imIdxR = imPair->imR();
                auto imL = m_projectData.image(imIdxL);
                auto imR = m_projectData.image(imIdxR);

                if (imL && imR) {
                    matToBind = utils::concatenateMat({imL->cvMat(),imR->cvMat()},utils::CONCAT_HORIZONTAL);
                }
            }
        }

        if (!matToBind.empty()) textureBind(matToBind);
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
        textureDisplay(newW, newH);
        ImGui::PopStyleVar();

        ImVec2 winPos = ImGui::GetWindowPos();
        const float posX = winPos.x + imStartX;
        const float posY = winPos.y + imStartY;

        if (isOneOf(m_currentTab,{Tab_General,Tab_Image})) {
            if (showFeatures) _showFeatures(ImVec2(posX, posY), ImVec2(newW, newH),col,featuresSize);
            if (m_projectData.image(m_currentImage)) {
                ImGui::SetCursorPosY(25);
                ImGui::Text("Number of features: %i", m_projectData.image(m_currentImage)->getNbFeatures());
            }
        } else if (m_currentTab == Tab_Stereo) {
            if (showMatches) _showMatches(ImVec2(posX, posY), ImVec2(newW, newH),col,1.0f);
            if (m_projectData.imagePair(m_currentImage)) {
                ImGui::SetCursorPosY(25);
                ImGui::Text("Number of matches: %i", m_projectData.imagePair(m_currentImage)->getNbMatches());
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
        ImGui::Text("--");
        ImGui::SameLine();
        ImGui::Checkbox("Show matches",&showMatches);
    }
    if (showFeatures || showMatches) {
        ImGui::SameLine();
        ImGui::ColorEdit4("MyColor##3", (float*)&col, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel );
        ImGui::SameLine();
    }
    ImGui::PopFont();
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

void Application::_showFeatures(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float featuresSize)
{
    auto im = m_projectData.image(m_currentImage);
    if (im && im->hasFeatures()){
        const auto & keyPts = im->getKeyPoints();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        for (const auto & kpt : keyPts) {
            float x = pos.x + kpt.pt.x / float(m_textureWidth) * size.x;
            float y = pos.y + kpt.pt.y / float(m_textureHeight) * size.y;

            draw_list->AddCircle(ImVec2(x, y), featuresSize,
                                 IM_COL32(col.x*255,col.y*255,col.z*255,col.w*255), 6, 2);
        }
    }
}

void Application::_showMatches(const ImVec2 &pos, const ImVec2 &size, const ImVec4 &col, float lineWidth)
{
    auto imPair = m_projectData.imagePair(m_currentImage);
    if (!imPair || !imPair->isValid()) return;
    if (!imPair->hasMatches()) return;

    auto imIdxL = imPair->imL();
    auto imIdxR = imPair->imR();
    auto imL = m_projectData.image(imIdxL);
    auto imR = m_projectData.image(imIdxR);

    if (!imL) return;
    if (!imR) return;
    if (!imL->hasFeatures()) return;
    if (!imR->hasFeatures()) return;

    // the texture is rendered started in {pos} with the size {size}
    float ratioLeftTotal = imL->cvMat().cols / float(imL->cvMat().cols + imR->cvMat().cols);
    float ratioRightTotal = (1.0f - ratioLeftTotal);

    if (imL->cvMat().rows != imR->cvMat().rows) {
        LOG_ERR("Showing matches is not supported yet for images with different height");
    }

    const float posXr = ratioLeftTotal * size.x;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    auto ptsL = imL->getKeyPoints();
    auto ptsR = imR->getKeyPoints();
    auto matches = imPair->getMatches();
    for (int i = 0; i < matches.size(); ++i) {
        const auto & id1 = matches[i].iPtL;
        const auto & id2 = matches[i].iPtR;

        float xl = pos.x + float(ptsL[id1].pt.x) / float(imL->cvMat().cols) * ratioLeftTotal * size.x;
        float yl = pos.y + float(ptsL[id1].pt.y) / float(imL->cvMat().rows) * size.y;
        float xr = pos.x + posXr + float(ptsR[id2].pt.x) / float(imR->cvMat().cols) * ratioRightTotal * size.x;
        float yr = pos.y + float(ptsR[id2].pt.y) / float(imR->cvMat().rows) * size.y;

        draw_list->AddLine(ImVec2(xl,yl),ImVec2(xr,yr),
                           IM_COL32(col.x*255,col.y*255,col.z*255,col.w*255),lineWidth);
    }

}
