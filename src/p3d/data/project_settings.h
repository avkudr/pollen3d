#pragma once

#include "p3d/core/core.h"
#include "p3d/core/serialization.h"

typedef int p3dSetting;
enum p3dSetting_
{
    p3dSetting_projectSettings   = 1000,
    p3dSetting_matcherCurAlg     = 1001,
    p3dSetting_matcherFilterCoef = 1002,
};


struct ProjectSettings : Serializable<ProjectSettings>{

    static int initMeta();

    ProjectSettings() {

    }

    ~ProjectSettings() {

    }

    enum Matcher{
        BruteForceL1,
        BruteForce,
        FlannBased
    };

    int matcherCurAlg       = Matcher::BruteForceL1;
    float matcherFilterCoef = 0.3f;
};
