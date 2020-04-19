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

    ProjectSettings() {}

    ~ProjectSettings() {}

    int sharedFeatExtractionPars{1};
    int sharedMatchingPars{1};

    static constexpr const char* className() { return "ProjectSettings"; }
    static constexpr P3D_ID_TYPE alias() { return entt::hashed_string{className()}; }
};
}  // namespace p3d
