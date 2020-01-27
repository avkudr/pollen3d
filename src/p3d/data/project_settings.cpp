#include "project_settings.h"

int dummyProjectSettings_ = ProjectSettings::initMeta();

int ProjectSettings::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        meta::reflect<ProjectSettings>(p3d_hash(p3dSetting_projectSettings))
                .data<&ProjectSettings::matcherCurAlg>(p3d_hash(p3dSetting_matcherCurAlg))
                .data<&ProjectSettings::matcherFilterCoef>(p3d_hash(p3dSetting_matcherFilterCoef));
    }
    return firstCall;
}
