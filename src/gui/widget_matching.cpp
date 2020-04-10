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
    if (!disabled) {
        const auto& iL = data.getImage(imPair->imL());
        const auto& iR = data.getImage(imPair->imR());
        disabled |= (iL.getNbFeatures() == 0);
        disabled |= (iR.getNbFeatures() == 0);
    }

    if (disabled) ImGuiC::PushDisabled();

    bool run{false}, runAll{false};

    if (ImGuiC::Collapsing("Matching", &run, &runAll)) {
        bool disableButtons = disabled;
        auto imL = data.imagePairL(m_currentItemIdx);
        auto imR = data.imagePairR(m_currentItemIdx);
        disableButtons = !((imL != nullptr) && imL->hasFeatures());
        disableButtons = !((imR != nullptr) && imR->hasFeatures());

        if (disableButtons) ImGuiC::PushDisabled();

        // **** parameters

        bool needsUpdate = false;
        auto matchingPars = imPair ? imPair->getMatchingPars() : MatchingPars();

        ImGuiC::BeginSubGroup();
        const char* matchingAlgos[] = {"FlannBased",
                                       "BruteForce",
                                       "BruteForce-L1",
                                       "BruteForce-Hamming",
                                       "BruteForce-HammmingLUT",
                                       "BruteForce-SL2"};
        auto matcherCurAlg = matchingPars.method;

        if (ImGui::Combo("matcher", &matcherCurAlg, matchingAlgos, IM_ARRAYSIZE(matchingAlgos))) {
            matchingPars.method = matcherCurAlg;
            needsUpdate = true;
        }

        auto matcherFilterCoef = matchingPars.filterCoeff;

        float min = 0.1f;
        float max = 1.0f;
        ImGui::InputFloat("filter coef", &matcherFilterCoef, min, max, "%.2f");
        matcherFilterCoef = std::max(min, std::min(matcherFilterCoef, max));
        if (ImGui::IsItemEdited()) {
            matchingPars.filterCoeff = matcherFilterCoef;
            needsUpdate = true;
        }

        bool globalSetting =
            ProjectManager::get()->getSetting(p3dSetting_shaderMatchingPars).cast<bool>();
        if (ImGui::Checkbox("shared parameters##matching", &globalSetting)) {
            if (globalSetting == true) {
                ProjectManager::get()->copyImagePairProperty(data, p3dImagePair_matchingPars,
                                                             m_currentItemIdx);
                CommandManager::get()->mergeNextCommand();
            }
            ProjectManager::get()->setSetting(p3dSetting_shaderMatchingPars, globalSetting);
        }

        if (needsUpdate) {
            std::vector<int> pairsToUpdate = {};
            if (!globalSetting) pairsToUpdate = {m_currentItemIdx};
            ProjectManager::get()->setImagePairProperty(data, p3dImagePair_matchingPars,
                                                        matchingPars, pairsToUpdate);
        }

        // **** tasks

        ImGui::Separator();
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
