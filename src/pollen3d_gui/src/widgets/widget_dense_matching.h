#pragma once

#include <imgui.h>

#include "p3d/data/project_data.h"
#include "widget.h"

class WidgetDenseMatching : public Widget
{
public:
    WidgetDenseMatching() {}

private:
    void drawImpl(p3d::ProjectData& data) override;
};
