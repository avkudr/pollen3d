#pragma once

#include <imgui.h>

#include "p3d/data/project_data.h"
#include "widget.h"

class WidgetFeatureExtract : public Widget
{
public:
    WidgetFeatureExtract() {}

private:
    void drawImpl(p3d::ProjectData& data) override;
};
