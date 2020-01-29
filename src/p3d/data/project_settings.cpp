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
                .data<&ProjectSettings::matcherFilterCoef>(P3D_ID_TYPE(p3dSetting_matcherFilterCoef));
    }
    return firstCall;
}
