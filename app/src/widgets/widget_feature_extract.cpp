#include "widget_feature_extract.h"

#include "p3d/core.h"
#include "p3d/data/project_settings.h"
#include "p3d/image/feature_extraction.h"
#include "p3d/project.h"
#include "p3d/tasks.h"

#include "../common/common.h"
#include "../common/imgui_custom.h"

using namespace p3d;

void WidgetFeatureExtract::drawImpl(p3d::Project& data)
{
    bool disabled = false;
    auto image = data.image(m_currentItemIdx);
    if (image == nullptr) disabled = true;

    if (disabled) ImGuiC::PushDisabled();

    bool run = false, runAll = false;
    if (ImGuiC::Collapsing("Feature extraction", &run, &runAll))
    {
        bool needsUpdate        = false;
        auto featExtractionPars =
            image ? image->getFeatExtractionPars() : FeatExtractionPars();

        ImGuiC::BeginSubGroup();
        ImGui::Text("AKAZE keypoint detector");
        ImGuiC::HelpMarker("Fast Explicit Diffusion for Accelerated Features in Nonlinear "
                           "Scale Spaces. Pablo F. Alcantarilla, Jesús Nuevo and Adrien "
                           "Bartoli. (BMVC 2013)");

        auto descSize = featExtractionPars.descSize;
        ImGui::InputInt("desc size", &descSize);
        if (ImGui::IsItemEdited())
        {
            featExtractionPars.descSize = descSize;
            needsUpdate                 = true;
        }
        ImGuiC::HoveredTooltip("Size of the descriptor in bits. 0 => Full size");

        auto descCh = featExtractionPars.descChannels;
        ImGui::InputInt("desc channels", &descCh);
        if (ImGui::IsItemEdited())
        {
            descCh                          = std::min(std::max(descCh, 1), 3);
            featExtractionPars.descChannels = descCh;
            needsUpdate                     = true;
        }
        ImGuiC::HoveredTooltip("Number of channels in the descriptor (1, 2, 3)");

        auto threshold = featExtractionPars.threshold;

        ImGui::InputFloat("threshold",
                          &threshold,
                          0.0002f,
                          0.0f,
                          "%0.5f",
                          ImGuiInputTextFlags_CharsScientific);
        if (ImGui::IsItemEdited())
        {
            featExtractionPars.threshold = threshold;
            needsUpdate                  = true;
        }
        ImGuiC::HoveredTooltip("Detector response threshold to accept point");

        // ***** are setting shared?

        const auto propID = p3dSetting_sharedFeatExtractionPars;
        bool globalSetting = p3d::getSetting(data, propID).cast<int>() > 0;
        if (ImGui::Checkbox("shared parameters##featextraction", &globalSetting)) {
            if (globalSetting == true) {
                p3d::copyImageProperty(data, P3D_ID_TYPE(p3dImage_featExtractionPars),
                                       m_currentItemIdx);
                p3d::mergeNextCommand();
            }
            int s = globalSetting ? 1 : 0;
            p3d::setSetting(data, propID, s);
        }

        if (ImGui::Button(P3D_ICON_RUN " Extract features")) run = true;
        ImGui::SameLine();
        if (ImGui::Button(P3D_ICON_RUNALL " ALL##extract_features")) runAll = true;

        // ***** smth changed?

        if (needsUpdate)
        {
            std::vector<int> imgsToUpdate = {};
            if (!globalSetting) imgsToUpdate = {m_currentItemIdx};

            p3d::setImageProperty(data,
                                  p3dImage_featExtractionPars,
                                  featExtractionPars,
                                  imgsToUpdate);
        }
        ImGuiC::EndSubGroup();
    }

    if (disabled) {
        ImGuiC::HoveredTooltip("Project has no images");
        ImGuiC::PopDisabled();
    }

    m_tasks.clear();
    if (run) m_tasks.insert("run");
    if (runAll) m_tasks.insert("run_all");
}
