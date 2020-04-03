#include "imgui_custom.h"
#include "imgui_internal.h"

void ImGuiC::PushDisabled()
{
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
}

void ImGuiC::PopDisabled()
{
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();
}


void ImGuiC::ItemCircle(const char *label, const ImVec2 & pt, float radius, const ImU32 & color32)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    const float size = radius*1.5f;
    const ImRect leftCircle(pt.x - size, pt.y - size, pt.x + size, pt.y + size);
    ImGui::ItemSize(leftCircle, 0);
    const ImGuiID id = window->GetID(label);
    ImGui::ItemAdd(leftCircle, id);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddCircle(pt, radius, color32, 6, 2);

}

void ImGuiC::HelpMarker(const char *desc, bool sameLine)
{
    if (sameLine) ImGui::SameLine();
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
    ImGui::PopItemFlag();
}
