#include "project_settings.h"

int dummyProjectSettings_ = ProjectSettings::initMeta();

int ProjectSettings::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        entt::meta<ProjectSettings>()
                .type(P3D_ID_TYPE(p3dSetting_projectSettings))
                .data<&ProjectSettings::matcherCurAlg>(P3D_ID_TYPE(p3dSetting_matcherCurAlg))
                .data<&ProjectSettings::matcherFilterCoef>(P3D_ID_TYPE(p3dSetting_matcherFilterCoef))
                .data<&ProjectSettings::featuresDescType>(P3D_ID_TYPE(p3dSetting_featuresDescType))
                .data<&ProjectSettings::featuresDescSize>(P3D_ID_TYPE(p3dSetting_featuresDescSize))
                .data<&ProjectSettings::featuresDescChannels>(P3D_ID_TYPE(p3dSetting_featuresDescChannels))
                .data<&ProjectSettings::featuresThreshold>(P3D_ID_TYPE(p3dSetting_featuresThreshold));
    }
    return firstCall;
}
