#include "widget_matching.h"

#include "p3d/core.h"
#include "p3d/project.h"
#include "p3d/tasks.h"

#include "../common/app_state.h"
#include "../common/common.h"
#include "../common/imgui_custom.h"
#include "../widgets/heavy_task.h"

using namespace p3d;

void WidgetMatching::drawImpl(Project& data)
{
    bool disabled = false;
    int currentPair = m_appState ? m_appState->itemIdx() : 0;
    auto imPair = data.imagePair(currentPair);
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
        auto imL = data.imagePairL(currentPair);
        auto imR = data.imagePairR(currentPair);
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

        // ***** are setting shared?

        const auto propID = p3dSetting_sharedMatchingPars;
        bool globalSetting = p3d::getSetting(data, propID).cast<int>() > 0;
        if (ImGui::Checkbox("shared parameters##matching", &globalSetting)) {
            if (globalSetting == true) {
                p3d::copyImagePairProperty(data, p3dImagePair_matchingPars,
                                           currentPair);
                p3d::mergeNextCommand();
            }
            int s = globalSetting ? 1 : 0;
            p3d::setSetting(data, propID, s);
        }

        // ***** smth changed?

        if (needsUpdate) {
            std::vector<int> pairsToUpdate = {};
            if (!globalSetting) pairsToUpdate = {currentPair};
            p3d::setImagePairProperty(data, p3dImagePair_matchingPars,
                                                        matchingPars, pairsToUpdate);
        }

        // **** tasks

        ImGui::Separator();
        run |= ImGui::Button(P3D_ICON_RUN " Match features");
        ImGui::SameLine();
        runAll |= ImGui::Button(P3D_ICON_RUNALL " ALL##matching");
        ImGuiC::EndSubGroup();
        if (disableButtons) ImGuiC::PopDisabled();
    }

    if (disabled) ImGuiC::PopDisabled();

    auto f = [&](const std::vector<int> & imgs = {}) {
        bool success = p3d::matchFeatures(data, imgs);
        if (success && m_appState) {
            m_appState->setSection(Section_Matches);
            m_appState->setTextureNeedsUpdate(true);
        }
    };

    if (run) {
        const std::vector<int> imgs = {currentPair};
        HeavyTask::run(f,imgs);
    } else if (runAll) {
        HeavyTask::run(f);
    }
}
