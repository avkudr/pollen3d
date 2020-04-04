#include "widget_feature_extract.h"

#include "p3d/core.h"
#include "p3d/data/project_data.h"
#include "p3d/project_manager.h"

#include "common.h"
#include "imgui_custom.h"

using namespace p3d;

void WidgetFeatureExtract::drawImpl(p3d::ProjectData& m_projectData)
{
    bool disabled = false;
    auto image = m_projectData.image(m_currentItemIdx);
    if (image == nullptr) disabled = true;

    if (disabled) ImGuiC::PushDisabled();

    m_run = m_runAll = false;
    if (ImGuiC::Collapsing("Feature extraction", &m_run, &m_runAll)) {
        auto v2 = ProjectManager::get()->settings().featuresDescSize;
        auto v3 = ProjectManager::get()->settings().featuresDescChannels;
        auto v4 = ProjectManager::get()->settings().featuresThreshold;

        ImGui::Text("AKAZE keypoint detector");
        ImGuiC::HelpMarker(
            "Fast Explicit Diffusion for Accelerated Features in Nonlinear "
            "Scale Spaces. Pablo F. Alcantarilla, Jesús Nuevo and Adrien "
            "Bartoli. (BMVC 2013)");

        ImGui::InputInt("desc size", &v2);
        if (ImGui::IsItemEdited())
            ProjectManager::get()->setSetting(p3dSetting_featuresDescSize, v2);
        ImGuiC::HelpMarker("Size of the descriptor in bits. 0 => Full size");

        ImGui::InputInt("desc channels", &v3);
        if (ImGui::IsItemEdited()) {
            v3 = std::min(std::max(v3, 1), 3);
            ProjectManager::get()->setSetting(p3dSetting_featuresDescChannels,
                                              v3);
        }
        ImGuiC::HelpMarker("Number of channels in the descriptor (1, 2, 3)");

        ImGui::InputFloat("threshold", &v4, 0.0002f, 0.0f, "%0.5f",
                          ImGuiInputTextFlags_CharsScientific);
        if (ImGui::IsItemEdited())
            ProjectManager::get()->setSetting(p3dSetting_featuresThreshold, v4);
        ImGuiC::HelpMarker("Detector response threshold to accept point");

        if (ImGui::Button(P3D_ICON_RUN " Extract features")) m_run = true;
        ImGui::SameLine();
        if (ImGui::Button(P3D_ICON_RUNALL " ALL##extract_features"))
            m_runAll = true;
    }
    if (disabled) ImGuiC::PopDisabled();
}
