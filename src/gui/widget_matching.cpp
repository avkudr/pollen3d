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
        bool disableButtons = disabled;
        auto imL = data.imagePairL(m_currentItemIdx);
        auto imR = data.imagePairR(m_currentItemIdx);
        disableButtons = !((imL != nullptr) && imL->hasFeatures());
        disableButtons = !((imR != nullptr) && imR->hasFeatures());

        if (disableButtons) ImGuiC::PushDisabled();

        ImGuiC::BeginSubGroup();
        const char* matchingAlgos[] = {"FlannBased",
                                       "BruteForce",
                                       "BruteForce-L1",
                                       "BruteForce-Hamming",
                                       "BruteForce-HammmingLUT",
                                       "BruteForce-SL2"};
        auto matcherCurAlg = imPair ? imPair->getMatchingPars().method : 2;

        if (ImGui::Combo("matcher", &matcherCurAlg, matchingAlgos,
                         IM_ARRAYSIZE(matchingAlgos))) {
            ProjectManager::get()->setProperty(
                imPair->matchingPars(), p3dMatching_method, matcherCurAlg);
        }

        auto matcherFilterCoef = imPair ? imPair->getMatchingPars().filterCoeff : 0.3f;

        float min = 0.1f;
        float max = 1.0f;
        ImGui::InputFloat("filter coef", &matcherFilterCoef, min, max, "%.2f");
        matcherFilterCoef = std::max(min, std::min(matcherFilterCoef, max));
        if (ImGui::IsItemEdited()) {
            ProjectManager::get()->setProperty(imPair->matchingPars(), p3dMatching_filterCoeff,
                                               matcherFilterCoef);
        }

        if (ImGui::Button("copy settings to all")) {
            ProjectManager::get()->copyImagePairProperty(
                data, p3dImagePair_matchingPars, m_currentItemIdx);
        }
        if (ImGui::Button(P3D_ICON_RUN " Match features")) run = true;
        ImGui::SameLine();
        if (ImGui::Button(P3D_ICON_RUNALL " ALL##matching")) runAll = true;
        ImGuiC::EndSubGroup();
        if (disableButtons) ImGuiC::PopDisabled();
    }

    if (disabled) ImGuiC::PopDisabled();

    m_tasks.clear();
    if (run)
        m_tasks.insert("run");
    else if (runAll)
        m_tasks.insert("run_all");
}
