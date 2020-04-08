#include "plots.h"

#include "imgui.h"

#include "p3d/core.h"
#include "p3d/data/project_data.h"

namespace p3d::plot
{
void ReprojectionError(const ProjectData& data, bool* open, int width)
{
    float w = static_cast<float>(width);
    if (open && *open) ImGui::OpenPopup("Reproj error##plot");
    if (ImGui::BeginPopupModal("Reproj error##plot", open,
                               ImGuiWindowFlags_Modal |
                                   ImGuiWindowFlags_NoScrollbar |
                                   ImGuiWindowFlags_NoResize)) {
        static float rngPx = 1.5;
        static float ptSize = 2.0;
        ImGui::SliderFloat("range(px)", &rngPx, 1.2f, 100.0f);
        ImGui::SliderFloat("point size(px)", &ptSize, 1.0f, 10.0f);

        if (ImGui::BeginChild("##actual_plot_reproj_error", ImVec2(w, w))) {
            ImVec2 a1 = ImGui::GetWindowPos();
            ImVec2 center = ImVec2(a1.x + w / 2.0f, a1.y + w / 2.0f);

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImU32 color = IM_COL32(255, 255, 255, 255 * 0.4f);
            draw_list->AddLine(ImVec2(center.x, center.y - w * 0.5f),
                               ImVec2(center.x, center.y + w * 0.5f), color);
            draw_list->AddLine(ImVec2(center.x - w * 0.5f, center.y),
                               ImVec2(center.x + w * 0.5f, center.y), color);
            draw_list->AddCircle(center, w * 0.5f * 0.5f / rngPx, color, 32, 1);
            draw_list->AddCircle(center, w * 0.5f * 1.0f / rngPx, color, 32, 1);

            if (!data.getPointCloudCtnr().contains("sparse")) {
                LOG_ERR(
                    "Plot of reprojection error needs a sparse point cloud");
            }
            auto pcdSparse = data.getPointCloudCtnr().at("sparse");
            const auto& pts3D = pcdSparse.getVertices();
            const auto& W = data.getMeasurementMatrixFull();
            const auto& P = data.getCameraMatricesMat();

            Mat4X X;
            X.setOnes(4, pts3D.cols());
            X.topRows(3) = pts3D.cast<double>();
            Mat error = W - P * X;

            int less1px = 0;
            int less05px = 0;
            int total = 0;
            int nbCams = P.rows() / 3;
            for (int c = 0; c < P.rows() / 3; ++c) {
                ImU32 color2 =
                    IM_COL32(0, 255 * c / float(nbCams), 255, 255 * 0.9f);
                for (int i = 0; i < error.cols(); ++i) {
                    if (error(3 * c + 2, i) != 0.0) continue;

                    p3d::Vec2f e = error.block(3 * c, i, 2, 1).cast<float>();
                    const auto enorm = e.norm();
                    if (enorm < 1.0f) less1px++;
                    if (enorm < 0.5f) less05px++;
                    total++;

                    e *= w * 0.5f / rngPx;
                    e(0) += center.x;
                    e(1) += center.y;
                    draw_list->AddCircle(ImVec2(e(0), e(1)), ptSize, color2, 4,
                                         1);
                }
            }

            ImGui::Text("From %i points", int(error.cols()));
            ImGui::Text("< 1px: %0.1f %%", 100.0f * less1px / float(total));
            ImGui::Text("< 0.5px: %0.1f %%", 100.0f * less05px / float(total));
            ImGui::EndChild();
        }
        ImGui::EndPopup();
    }
}

}  // namespace p3d::plot
