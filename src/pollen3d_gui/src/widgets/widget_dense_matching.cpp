#include "widget_dense_matching.h"

#include "p3d/core.h"
#include "p3d/data/project_data.h"
#include "p3d/project_manager.h"

#include "../common/common.h"
#include "../common/imgui_custom.h"

using namespace p3d;

void WidgetDenseMatching::drawImpl(ProjectData& data)
{
    bool disabled = false;
    auto imPair = data.imagePair(m_currentItemIdx);
    if (imPair == nullptr)
        disabled = true;
    else if (!imPair->isRectified())
        disabled = true;

    if (disabled) ImGuiC::PushDisabled();

    bool run{false}, runAll{false}, runBilateral{false}, runBilateralAll{false},
        runFilterSpeckles{false}, runFilterSpecklesAll{false};

    if (ImGuiC::Collapsing("Dense matching", &run, &runAll)) {
        ImGuiC::BeginSubGroup();

        const char* denseMatchingAlgos[] = {"SGBM", "HH", "SGBM_3WAY"};
        auto matcherCurAlg =
            imPair ? imPair->getDenseMatchingPars().dispMethod : 0;

        if (ImGui::Combo("method", &matcherCurAlg, denseMatchingAlgos,
                         IM_ARRAYSIZE(denseMatchingAlgos))) {
            ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                               p3dDense_DispMethod,
                                               matcherCurAlg);
        }

        {
            auto dispLowerBound =
                imPair ? imPair->getDenseMatchingPars().dispLowerBound : -1;
            auto dispUpperBound =
                imPair ? imPair->getDenseMatchingPars().dispUpperBound : 2;
            auto dispBlockSize =
                imPair ? imPair->getDenseMatchingPars().dispBlockSize : 9;

            ImGui::InputInt("lowew bound", &dispLowerBound);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_DispLowerBound,
                                                   dispLowerBound);
            ImGui::InputInt("upper bound", &dispUpperBound);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_DispUpperBound,
                                                   dispUpperBound);
            ImGui::InputInt("block size", &dispBlockSize, 2, 2);
            if (ImGui::IsItemEdited()) {
                if (dispBlockSize % 2 == 0) dispBlockSize--;
                dispBlockSize = std::min(dispBlockSize, 21);
                dispBlockSize = std::max(dispBlockSize, 1);
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_DispBlockSize,
                                                   dispBlockSize);
            }
        }

        if (ImGui::Button("Dense match")) run = true;
        ImGui::SameLine();
        if (ImGui::Button("ALL##densematch")) runAll = true;
        ImGui::Separator();

        bool disableButtons = disabled || !imPair->hasDisparityMap();
        if (disableButtons) ImGuiC::PushDisabled();

        ImGui::Text("Bilateral filter: ");
        {
            auto v1 = imPair ? imPair->getDenseMatchingPars().bilateralD : 9;
            auto v2 = imPair
                          ? imPair->getDenseMatchingPars().bilateralSigmaColor
                          : 180;
            auto v3 = imPair
                          ? imPair->getDenseMatchingPars().bilateralSigmaSpace
                          : 180;

            ImGui::InputInt("diameter", &v1);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_BilateralD, v1);
            ImGuiC::HelpMarker(
                "Diameter of each pixel neighborhood that is used during "
                "filtering. "
                "If it is non-positive, it is computed from sigmaSpace.");

            ImGui::InputInt("sigma color", &v2);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_BilateralSigmaColor,
                                                   v2);
            ImGuiC::HelpMarker(
                "Filter sigma in the color space. A larger value of the "
                "parameter means that "
                "farther colors within the pixel neighborhood (see sigmaSpace) "
                "will be mixed "
                "together, resulting in larger areas of semi-equal color.");

            ImGui::InputInt("sigma space", &v3);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_BilateralSigmaSpace,
                                                   v3);
            ImGuiC::HelpMarker(
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

#ifdef POLLEN3D_DEBUG
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
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_DispFilterNewValue,
                                                   dispFilterNewValue);
            ImGui::InputInt("max speckle size", &dispFilterMaxSpeckleSize);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(
                    imPair->denseMatchingPars(),
                    p3dDense_DispFilterMaxSpeckleSize,
                    dispFilterMaxSpeckleSize);
            ImGui::InputInt("max diff", &dispFilterMaxDiff);
            if (ImGui::IsItemEdited())
                ProjectManager::get()->setProperty(imPair->denseMatchingPars(),
                                                   p3dDense_DispFilterMaxDiff,
                                                   dispFilterMaxDiff);
        }

        if (ImGui::Button("Filter##speckles")) runFilterSpeckles = true;
        ImGui::SameLine();
        if (ImGui::Button("ALL##speckles")) runFilterSpecklesAll = true;
#endif

        if (disableButtons) ImGuiC::PopDisabled();
        ImGuiC::EndSubGroup();
    }

    if (disabled) ImGuiC::PopDisabled();

    m_tasks.clear();
    if (run)
        m_tasks.insert("run");
    else if (runAll)
        m_tasks.insert("run_all");
    else if (runBilateral)
        m_tasks.insert("run_bilateral");
    else if (runBilateralAll)
        m_tasks.insert("run_bilateral_all");
    else if (runFilterSpeckles)
        m_tasks.insert("run_filter_speckles");
    else if (runFilterSpecklesAll)
        m_tasks.insert("run_filter_speckles_all");
}
