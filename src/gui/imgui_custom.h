#pragma once

#include "imgui.h"

namespace ImGuiC
{  // C for custom

void PushDisabled(bool disable = true);
void PopDisabled();

void ItemCircle(const char* label, const ImVec2& pt, float radius,
                const ImU32& color);
void HelpMarker(const char* desc, bool sameLine = true);
void HoveredTooltip(const char* desc);

bool Collapsing(const char* label, bool* btnRun, bool* btnRunAll = nullptr);
bool RunButton(ImGuiID id, const ImVec2& pos, const char* icon);

void BeginSubGroup();
void EndSubGroup();

void ButtonTooltip(const char* text);

}  // namespace ImGuiC
