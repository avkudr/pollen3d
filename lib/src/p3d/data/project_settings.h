#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dSetting;
enum P3D_API p3dSetting_ {
    p3dSetting_sharedFeatExtractionPars,
    p3dSetting_sharedMatchingPars,
};

struct P3D_API ProjectSettings : Serializable<ProjectSettings> {
    static int initMeta();
    static const char* classNameStatic() { return "ProjectSettings"; }

    ProjectSettings() {}

    ~ProjectSettings() {}

    int sharedFeatExtractionPars{1};
    int sharedMatchingPars{1};

};
}  // namespace p3d
