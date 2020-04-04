#pragma once

#include <set>

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

    bool isRequested(const std::string& task) const
    {
        return m_tasks.count(task) > 0;
    }

protected:
    virtual void drawImpl(p3d::ProjectData& data) = 0;

    int m_currentItemIdx{0};
    std::set<std::string> m_tasks;
};
