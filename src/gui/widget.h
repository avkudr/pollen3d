#pragma once

#include "p3d/core.h"
#include "p3d/data/project_data.h"

class Widget
{
public:
    virtual ~Widget() {}

    void draw(p3d::ProjectData& data, int currentItem = 0)
    {
        m_currentItemIdx = currentItem;
        drawImpl(data);
    }

    bool runRequested() const { return m_run; }
    bool runAllRequested() const { return m_runAll; }

protected:
    virtual void drawImpl(p3d::ProjectData& data) = 0;

    int m_currentItemIdx{0};
    bool m_run{false};
    bool m_runAll{false};
};
