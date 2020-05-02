#include "project_settings.h"

using namespace p3d;

int dummyProjectSettings_ = ProjectSettings::initMeta();

int ProjectSettings::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        SERIALIZED_ADD_READ_WRITE(ProjectSettings);
        entt::meta<ProjectSettings>()
            .data<&ProjectSettings::sharedFeatExtractionPars>(
                P3D_ID_TYPE(p3dSetting_sharedFeatExtractionPars))
            .data<&ProjectSettings::sharedMatchingPars>(
                P3D_ID_TYPE(p3dSetting_sharedMatchingPars));
    }
    return firstCall;
}
