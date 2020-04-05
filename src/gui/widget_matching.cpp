#include "widget_matching.h"

#include "p3d/core.h"
#include "p3d/data/project_data.h"
#include "p3d/project_manager.h"

#include "common.h"
#include "imgui_custom.h"

using namespace p3d;

void WidgetMatching::drawImpl(ProjectData& data)
{
    bool disabled = false;
    auto imPair = data.imagePair(m_currentItemIdx);
    if (imPair == nullptr) disabled = true;
    //    else if (!imPair->has())
    //        disabled = true;

    if (disabled) ImGuiC::PushDisabled();

    bool run{false}, runAll{false};

    if (ImGuiC::Collapsing("Matching", &run, &runAll)) {
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        p0.x += 5;
        p0.y += 5;
        ImGui::SetCursorScreenPos(p0);

        bool disableButtons = disabled;
        auto imL = data.imagePairL(m_currentItemIdx);
        auto imR = data.imagePairR(m_currentItemIdx);
        disableButtons = !((imL != nullptr) && imL->hasFeatures());
        disableButtons = !((imR != nullptr) && imR->hasFeatures());

        if (disableButtons) ImGuiC::PushDisabled();

        ImGui::BeginGroup();
        const char* matchingAlgos[] = {"BruteForce-L1", "BruteForce",
                                       "FlannBased"};
        auto matcherCurAlg = ProjectManager::get()
                                 ->getSetting(p3dSetting_matcherCurAlg)
                                 .cast<int>();

        if (ImGui::Combo("matcher", &matcherCurAlg, matchingAlgos,
                         IM_ARRAYSIZE(matchingAlgos))) {
            ProjectManager::get()->setSetting(p3dSetting_matcherCurAlg,
                                              matcherCurAlg);
        }

        {
            auto matcherFilterCoef =
                ProjectManager::get()
                    ->getSetting(p3dSetting_matcherFilterCoef)
                    .cast<float>();
            float min = 0.1f;
            float max = 1.0f;
            ImGui::InputFloat("filter coef", &matcherFilterCoef, min, max,
                              "%.2f");
            matcherFilterCoef = std::max(min, std::min(matcherFilterCoef, max));
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setSetting(p3dSetting_matcherFilterCoef,
                                                  matcherFilterCoef);
        }

        if (ImGui::Button("Match features")) run = true;
        ImGui::SameLine();
        if (ImGui::Button("ALL##matching")) runAll = true;
        ImGui::Dummy(ImVec2(0.0f, 10.0f));
        ImGui::EndGroup();
        if (disableButtons) ImGuiC::PopDisabled();
    }

    if (disabled) ImGuiC::PopDisabled();

    m_tasks.clear();
    if (run)
        m_tasks.insert("run");
    else if (runAll)
        m_tasks.insert("run_all");
}