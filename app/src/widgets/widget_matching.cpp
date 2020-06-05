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
    bool disabled = data.imagePairs().size() == 0;
    if (disabled) ImGuiC::PushDisabled();

    bool runAll{false};

    if (ImGuiC::Collapsing("Matching", nullptr, &runAll)) {
        bool disableButtons = disabled;
        if (disableButtons) ImGuiC::PushDisabled();

        // **** parameters

        bool needsUpdate = false;

        ImGuiC::BeginSubGroup();
        const char* matchingAlgos[] = {"FlannBased",
                                       "BruteForce",
                                       "BruteForce-L1",
                                       "BruteForce-Hamming",
                                       "BruteForce-HammmingLUT",
                                       "BruteForce-SL2"};

        auto matcherCurAlg = p3d::getSetting(data, p3dSetting_matchingMethod).cast<int>();
        if (ImGui::Combo("matcher", &matcherCurAlg, matchingAlgos, IM_ARRAYSIZE(matchingAlgos))) {
            p3d::setSetting(data, p3dSetting_matchingMethod, matcherCurAlg);
        }

        auto matcherFilterCoef =
            p3d::getSetting(data, p3dSetting_matchingFilterCoeff).cast<float>();

        float min = 0.1f;
        float max = 1.0f;
        ImGui::InputFloat("filter coef", &matcherFilterCoef, min, max, "%.2f");
        matcherFilterCoef = std::max(min, std::min(matcherFilterCoef, max));
        if (ImGui::IsItemEdited()) {
            p3d::setSetting(data, p3dSetting_matchingFilterCoeff, matcherFilterCoef);
        }

        // ***** are setting shared?

        /*  // delete (05/06/2020)
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
        */

        // **** tasks

        runAll |= ImGui::Button(P3D_ICON_RUNALL " Match features");
        ImGuiC::EndSubGroup();
        if (disableButtons) ImGuiC::PopDisabled();
    }

    if (disabled) ImGuiC::PopDisabled();

    auto f = [&](const std::vector<int>& imgs = {}) {
        bool success = p3d::matchFeatures(data);
        if (success && m_appState) {
            // m_appState->setSection(Section_Matches);
            // m_appState->setTextureNeedsUpdate(true);
        }
    };

    if (runAll) { HeavyTask::run(f); }
}
