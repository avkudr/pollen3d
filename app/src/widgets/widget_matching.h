#pragma once

#include <imgui.h>

#include "p3d/project.h"
#include "widget.h"

class WidgetMatching : public Widget
{
public:
    WidgetMatching() {}

private:
    void drawImpl(p3d::Project& data) override;
};
