#include "project_settings.h"

using namespace p3d;

int dummyProjectSettings_ = ProjectSettings::initMeta();

int ProjectSettings::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        entt::meta<ProjectSettings>()
            .alias(P3D_ID_TYPE(p3dSetting_projectSettings))
            .data<&ProjectSettings::featuresDescType>(P3D_ID_TYPE(p3dSetting_featuresDescType))
            .data<&ProjectSettings::featuresDescSize>(P3D_ID_TYPE(p3dSetting_featuresDescSize))
            .data<&ProjectSettings::featuresDescChannels>(
                P3D_ID_TYPE(p3dSetting_featuresDescChannels))
            .data<&ProjectSettings::featuresThreshold>(P3D_ID_TYPE(p3dSetting_featuresThreshold))
            .data<&ProjectSettings::sharedMatchingPars>(P3D_ID_TYPE(p3dSetting_shaderMatchingPars));
    }
    return firstCall;
}
