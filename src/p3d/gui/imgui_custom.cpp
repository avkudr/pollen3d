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


