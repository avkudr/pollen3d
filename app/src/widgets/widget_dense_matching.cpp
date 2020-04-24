#include "widget_dense_matching.h"

#include "p3d/core.h"
#include "p3d/project.h"
#include "p3d/tasks.h"

#include "../common/app_state.h"
#include "../common/common.h"
#include "../common/imgui_custom.h"
#include "../widgets/heavy_task.h"

using namespace p3d;

void WidgetDenseMatching::drawImpl(Project& data)
{
    bool disabled = false;
    int currentPair = m_appState ? m_appState->itemIdx() : 0;
    auto imPair = data.imagePair(currentPair);
    if (imPair == nullptr)
        disabled = true;
    else if (!imPair->isRectified())
        disabled = true;

    if (disabled) ImGuiC::PushDisabled();

    bool run{false}, runAll{false}, runBilateral{false}, runBilateralAll{false},
        runFilterSpeckles{false}, runFilterSpecklesAll{false};

    if (ImGuiC::Collapsing("Dense matching", &run, &runAll)) {
        bool needsUpdate = false;
        auto denseMatchingPars =
            imPair ? imPair->getDenseMatchingPars() : DenseMatchingPars();

        auto matcherCurAlg = denseMatchingPars.dispMethod;
        const char* denseMatchingAlgos[] = {"SGBM", "HH", "SGBM_3WAY"};
        ImGuiC::BeginSubGroup();

        {
            if (ImGui::Combo("method", &matcherCurAlg, denseMatchingAlgos,
                             IM_ARRAYSIZE(denseMatchingAlgos))) {
                denseMatchingPars.dispMethod = matcherCurAlg;
                needsUpdate = true;
            }
            ImGuiC::HoveredTooltip("Dense matching methods");
        }

        {
            auto dispLowerBound = denseMatchingPars.dispLowerBound;
            auto dispUpperBound = denseMatchingPars.dispUpperBound;
            auto dispBlockSize = denseMatchingPars.dispBlockSize;

            ImGui::InputInt("lower bound", &dispLowerBound);
            if (ImGui::IsItemEdited()) {
                denseMatchingPars.dispLowerBound = dispLowerBound;
                needsUpdate = true;
            }
            ImGuiC::HoveredTooltip(
                "lower bound.\n"
                "Minimum possible disparity value. Normally, it is zero but sometimes "
                "rectification algorithms can shift images, so this parameter needs to "
                "be adjusted accordingly.");

            ImGui::InputInt("upper bound", &dispUpperBound);
            if (ImGui::IsItemEdited()) {
                denseMatchingPars.dispUpperBound = dispUpperBound;
                needsUpdate = true;
            }
            ImGuiC::HoveredTooltip(
                "upperd bound.\n"
                "Maximum possible disparity value");

            ImGui::InputInt("block size", &dispBlockSize, 2, 2);
            if (ImGui::IsItemEdited()) {
                if (dispBlockSize % 2 == 0) dispBlockSize--;
                dispBlockSize = std::min(dispBlockSize, 21);
                dispBlockSize = std::max(dispBlockSize, 1);

                denseMatchingPars.dispBlockSize = dispBlockSize;
                needsUpdate = true;
            }

            ImGuiC::HoveredTooltip(
                "Matched block size.\n"
                "It must be an odd number >=1 . Normally, it should be somewhere in the "
                "3..11 range.");
        }

        if (ImGui::Button("Dense match")) run = true;
        ImGui::SameLine();
        if (ImGui::Button("ALL##densematch")) runAll = true;

        bool disableButtons = disabled || !imPair->hasDisparityMap();
        if (disableButtons) ImGuiC::PushDisabled();

#ifdef POLLEN3D_DEBUG
/*
        ImGui::Separator();
        ImGui::Text("Bilateral filter: ");
        {
            auto bilateralD = denseMatchingPars.bilateralD;
            auto sigmaColor = denseMatchingPars.bilateralSigmaColor;
            auto sigmaSpace = denseMatchingPars.bilateralSigmaSpace;

            ImGui::InputInt("diameter", &bilateralD);
            if (ImGui::IsItemEdited()) {
                denseMatchingPars.bilateralD = bilateralD;
                needsUpdate = true;
            }
            ImGuiC::HoveredTooltip(
                "Diameter of each pixel neighborhood that is used during "
                "filtering. "
                "If it is non-positive, it is computed from sigmaSpace.");

            ImGui::InputInt("sigma color", &sigmaColor);
            if (ImGui::IsItemEdited()) {
                denseMatchingPars.bilateralSigmaColor = sigmaColor;
                needsUpdate = true;
            }
            ImGuiC::HoveredTooltip(
                "Filter sigma in the color space. A larger value of the "
                "parameter means that "
                "farther colors within the pixel neighborhood (see sigmaSpace) "
                "will be mixed "
                "together, resulting in larger areas of semi-equal color.");

            ImGui::InputInt("sigma space", &sigmaSpace);
            if (ImGui::IsItemEdited()) {
                denseMatchingPars.bilateralSigmaSpace = sigmaSpace;
                needsUpdate = true;
            }
            ImGuiC::HoveredTooltip(
                "Filter sigma in the coordinate space. A larger value of the "
                "parameter means that "
                "farther pixels will influence each other as long as their "
                "colors are close enough (see sigmaColor"
                "). When diameter>0, it specifies the neighborhood size "
                "regardless of sigmaSpace. Otherwise, d is"
                "proportional to sigmaSpace.");
        }

        if (ImGui::Button("Filter##bilateral")) runBilateral = true;
        ImGui::SameLine();
        if (ImGui::Button("ALL##bilateral")) runBilateralAll = true;

        ImGui::Separator();
        ImGui::Text("Speckles filter: ");
        {
            auto dispFilterNewValue =
                imPair ? imPair->getDenseMatchingPars().dispFilterNewValue : 0;
            auto dispFilterMaxSpeckleSize =
                imPair ? imPair->getDenseMatchingPars().dispFilterMaxSpeckleSize
                       : 260;
            auto dispFilterMaxDiff =
                imPair ? imPair->getDenseMatchingPars().dispFilterMaxDiff : 10;

            ImGui::InputInt("new value", &dispFilterNewValue);
            if (ImGui::IsItemEdited())
                ;
            //                p3d::setProperty(imPair->denseMatchingPars(),
            // p3dDense_DispFilterNewValue,
            //                                                   dispFilterNewValue);
            ImGui::InputInt("max speckle size", &dispFilterMaxSpeckleSize);
            if (ImGui::IsItemEdited())
                ;
            //                p3d::setProperty(
            //                    imPair->denseMatchingPars(),
            //                    p3dDense_DispFilterMaxSpeckleSize,
            //                    dispFilterMaxSpeckleSize);
            ImGui::InputInt("max diff", &dispFilterMaxDiff);
            if (ImGui::IsItemEdited())
                ;
            //                p3d::setProperty(imPair->denseMatchingPars(),
            // p3dDense_DispFilterMaxDiff,
            //                                                   dispFilterMaxDiff);
        }

        if (ImGui::Button("Filter##speckles")) runFilterSpeckles = true;
        ImGui::SameLine();
        if (ImGui::Button("ALL##speckles")) runFilterSpecklesAll = true;
*/
#endif

        bool globalSetting = false;
        if (needsUpdate) {
            std::vector<int> pairsToUpdate = {};
            if (!globalSetting) pairsToUpdate = {currentPair };
            p3d::setImagePairProperty(data, p3dImagePair_denseMatchingPars,
                                      denseMatchingPars, pairsToUpdate);
        }

        if (disableButtons) ImGuiC::PopDisabled();
        ImGuiC::EndSubGroup();
    }

    if (disabled) ImGuiC::PopDisabled();

    auto fDisp = [&](const std::vector<int> &imIds = {}) {
        bool success = p3d::findDisparityMap(data, imIds);
        if (success && m_appState) {
            m_appState->setSection(Section_DisparityMap);
            m_appState->setTextureNeedsUpdate(true);
        }
    };

    auto fBilateral = [&](const std::vector<int> &imIds = {}) {
        p3d::filterDisparityBilateral(data, imIds);
        m_appState->setTextureNeedsUpdate(true);
    };

    auto fSpeckles = [&](const std::vector<int> &imIds = {}) {
        p3d::filterDisparitySpeckles(data, imIds);
        m_appState->setTextureNeedsUpdate(true);
    };

    const std::vector<int> imgs = {currentPair };

    if      (run) {                  HeavyTask::run(fDisp,imgs); }
    else if (runAll) {               HeavyTask::run(fDisp); }
    else if (runBilateral) {         HeavyTask::run(fBilateral,imgs); }
    else if (runBilateralAll) {      HeavyTask::run(fBilateral); }
    else if (runFilterSpeckles) {    HeavyTask::run(fSpeckles,imgs); }
    else if (runFilterSpecklesAll) { HeavyTask::run(fSpeckles); }
}
