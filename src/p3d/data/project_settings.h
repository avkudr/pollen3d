#pragma once

#include "p3d/core/core.h"

struct ProjectSettings{

    ProjectSettings() {
        static bool firstCall = true;
        if (firstCall) {
            firstCall = false;
            meta::reflect<ProjectSettings>(p3d_hash(p3dSetting_projectSettings))
                    .data<&ProjectSettings::matcherCurAlg>(p3d_hash(p3dSetting_matcherCurAlg))
                    .data<&ProjectSettings::matcherFilterCoef>(p3d_hash(p3dSetting_matcherFilterCoef));
        }
    }

    enum Matcher{
        BruteForceL1,
        BruteForce,
        FlannBased
    };

    int matcherCurAlg       = Matcher::BruteForceL1;
    float matcherFilterCoef = 0.3f;
};
