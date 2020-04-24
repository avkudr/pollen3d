#pragma once

#include <set>

#include "p3d/core.h"
#include "p3d/project.h"

class AppState;

class Widget
{
public:
    Widget(AppState * appState) : m_appState(appState)
    {
        m_appState = appState;
    }
    virtual ~Widget() {}

    void draw(p3d::Project& data)
    {
        drawImpl(data);
    }

protected:
    virtual void drawImpl(p3d::Project& data) = 0;

    AppState * m_appState{nullptr};
};
