#pragma once

#include <imgui.h>

#include "p3d/project.h"
#include "widget.h"

class WidgetDenseMatching : public Widget
{
public:
    WidgetDenseMatching(AppState * appState) : Widget(appState) {}

private:
    void drawImpl(p3d::Project& data) override;
};
