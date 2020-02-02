#pragma once

#include "imgui.h"

namespace ImGuiC{ // C for custom

    void PushDisabled();
    void PopDisabled();

    void ItemCircle(const char * label, const ImVec2 & pt, float radius, const ImU32 & color);
}
