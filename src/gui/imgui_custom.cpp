#include "imgui_custom.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui_internal.h"

#include <string>

#include "common.h"

void ImGuiC::PushDisabled(bool disable)
{
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, disable);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, disable ? 0.6f : 1.0f);
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

bool ImGuiC::Collapsing(const char *label, bool *btnRun, bool *btnRunAll)
{
    ImGuiWindow *window = ImGui::GetCurrentWindow();
    if (window->SkipItems) return false;

    // if (btnRun && !*btnRun) return false;

    ImGuiID id = window->GetID(label);
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_CollapsingHeader;
    if (btnRun || btnRunAll)
        flags |= ImGuiTreeNodeFlags_AllowItemOverlap |
                 ImGuiTreeNodeFlags_ClipLabelForTrailingButton;

    ImGuiC::PushDisabled(false);
    bool is_open = ImGui::TreeNodeBehavior(id, flags, label);
    ImGuiC::PopDisabled();
    if (btnRun || btnRunAll) {
        // Create a small overlapping close button
        // FIXME: We can evolve this into user accessible helpers to add extra
        // buttons on title bars, headers, etc.
        // FIXME: CloseButton can overlap into text, need find a way to clip the
        // text somehow.
        ImGuiContext &g = *GImGui;
        ImGuiItemHoveredDataBackup last_item_backup;
        float button_size = g.FontSize;

        auto LastItemRectMinx = window->DC.LastItemRect.Min.x;
        auto LastItemRectMaxx = window->DC.LastItemRect.Max.x;
        float button_y = window->DC.LastItemRect.Min.y;

        float button1_x = ImMax(
            LastItemRectMinx,
            LastItemRectMaxx - g.Style.FramePadding.x * 2.0f - button_size);

        float button2_x = ImMax(
            LastItemRectMinx, LastItemRectMaxx - g.Style.FramePadding.x * 3.0f -
                                  2.0f * button_size);
        if (btnRun) {
            float button_x = btnRunAll ? button2_x : button1_x;
            if (RunButton(window->GetID((void *)((intptr_t)id + 1)),
                          ImVec2(button_x, button_y), P3D_ICON_RUN)) {
                *btnRun = true;
            } else
                *btnRun = false;
        }

        if (btnRunAll) {
            if (RunButton(window->GetID((void *)((intptr_t)id + 2)),
                          ImVec2(button1_x, button_y), P3D_ICON_RUNALL)) {
                *btnRunAll = true;
            } else
                *btnRunAll = false;
        }

        last_item_backup.Restore();
    }

    return is_open;
}

bool ImGuiC::RunButton(ImGuiID id, const ImVec2 &pos,
                       const char *icon)  //, float size)
{
    using namespace ImGui;

    ImGuiContext &g = *GImGui;
    ImGuiWindow *window = g.CurrentWindow;

    // We intentionally allow interaction when clipped so that a mechanical
    // Alt,Right,Validate sequence close a window. (this isn't the regular
    // behavior of buttons, but it doesn't affect the user much because
    // navigation tends to keep items visible).
    const ImRect bb(pos, pos + ImVec2(g.FontSize, g.FontSize) +
                             g.Style.FramePadding * 2.0f);
    bool is_clipped = !ItemAdd(bb, id);

    bool hovered, held;
    bool pressed = ButtonBehavior(bb, id, &hovered, &held);
    if (is_clipped) return pressed;

    // Render
    ImU32 col =
        GetColorU32(held ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered);
    ImVec2 center = bb.GetCenter();
    auto radius = ImMax(2.0f, g.FontSize * 0.5f + 1.0f);
    if (hovered) {
        window->DrawList->AddCircleFilled(center, radius, col, 12);
        if (icon == P3D_ICON_RUN)
            ImGui::SetTooltip(
                "Run the task for current item (image, pair, etc.)");
        else
            ImGui::SetTooltip("Run the task for all items (images, pairs)");
    }

    float fontScale = 0.8;
    float cross_extent = g.FontSize * 0.5f * 0.7071f - 1.0f;
    ImU32 cross_col = GetColorU32(ImGuiCol_Text);
    center += ImVec2(1.0 - g.FontSize * fontScale * 0.5f,
                     -g.FontSize * fontScale * 0.5f);
    auto font = ImGui::GetFont();
    window->DrawList->AddText(font, g.FontSize * 0.8, center, cross_col, icon,
                              icon + strlen(icon));
    return pressed;
}

void ImGuiC::HoveredTooltip(const char *desc)
{
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
    ImGui::PopItemFlag();
}
