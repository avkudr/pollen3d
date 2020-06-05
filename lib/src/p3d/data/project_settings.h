#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"
#include "p3d/stereo/matching.h"

namespace p3d
{
typedef int p3dSetting;
enum P3D_API p3dSetting_ {
    p3dSetting_sharedFeatExtractionPars,
    p3dSetting_sharedMatchingPars,
    p3dSetting_matchingMethod,
    p3dSetting_matchingFilterCoeff,
};

struct P3D_API ProjectSettings : Serializable<ProjectSettings> {
    static int initMeta();
    static const char* classNameStatic() { return "ProjectSettings"; }

    ProjectSettings() {}

    ~ProjectSettings() {}

    int sharedFeatExtractionPars{1};
    int sharedMatchingPars{1};

    int matchingMethod{MatchingMethod_BRUTEFORCE_L1};
    float matchingFilterCoeff{0.3f};
};
}  // namespace p3d
