#pragma once

#include "p3d/core/core.h"
#include "p3d/core/serialization.h"

typedef int p3dSetting;
enum p3dSetting_
{
    p3dSetting_projectSettings   = 1000,
    p3dSetting_matcherCurAlg     = 1001,
    p3dSetting_matcherFilterCoef = 1002,

    p3dSetting_featuresDescType  = 1100,
    p3dSetting_featuresDescSize,
    p3dSetting_featuresDescChannels,
    p3dSetting_featuresThreshold,
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

    int matcherCurAlg{Matcher::BruteForceL1};
    float matcherFilterCoef{0.3f};

    int featuresDescType{5}; // cv::AKAZE::DESCRIPTOR_MLDB
    int featuresDescSize{0};
    int featuresDescChannels{3};
    float featuresThreshold{0.001f};
};
