#pragma once

#include "imgui.h"

namespace ImGuiC{ // C for custom

    void PushDisabled();
    void PopDisabled();

    void ItemCircle(const char * label, const ImVec2 & pt, float radius, const ImU32 & color);
    void HelpMarker(const char* desc, bool sameLine = true);

    bool Collapsing(const char* label, bool* btnRun, bool* btnRunAll);
    bool RunButton(ImGuiID id, const ImVec2& pos, const char* icon);
}
