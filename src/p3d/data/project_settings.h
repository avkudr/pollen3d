#pragma once

#include "p3d/core/core.h"
#include "p3d/core/serialization.h"

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
