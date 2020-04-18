#pragma once

#include <imgui.h>

#include "p3d/project.h"
#include "widget.h"

class WidgetFeatureExtract : public Widget
{
public:
    WidgetFeatureExtract() {}

private:
    void drawImpl(p3d::Project& data) override;
};
