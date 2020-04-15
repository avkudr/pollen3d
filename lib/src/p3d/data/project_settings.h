#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dSetting;
enum P3D_EXPORTS p3dSetting_ {
    p3dSetting_projectSettings = 1000,

    p3dSetting_featuresDescType = 1100,
    p3dSetting_featuresDescSize = 1101,
    p3dSetting_featuresDescChannels = 1102,
    p3dSetting_featuresThreshold = 1103,

    p3dSetting_shaderMatchingPars = 1200,
};

P3D_EXPORTS struct ProjectSettings : Serializable<ProjectSettings> {
    static int initMeta();

    ProjectSettings() {}

    ~ProjectSettings() {}

    int featuresDescType{5};  // cv::AKAZE::DESCRIPTOR_MLDB
    int featuresDescSize{0};
    int featuresDescChannels{3};
    float featuresThreshold{0.001f};

    bool sharedMatchingPars{true};
};
}  // namespace p3d
