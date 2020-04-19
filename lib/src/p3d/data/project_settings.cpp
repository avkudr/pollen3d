#include "project_settings.h"

using namespace p3d;

int dummyProjectSettings_ = ProjectSettings::initMeta();

int ProjectSettings::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        entt::meta<ProjectSettings>()
            .alias(alias())
            .data<&ProjectSettings::sharedFeatExtractionPars>(
                P3D_ID_TYPE(p3dSetting_sharedFeatExtractionPars))
            .data<&ProjectSettings::sharedMatchingPars>(P3D_ID_TYPE(p3dSetting_sharedMatchingPars));

        SERIALIZED_ADD_READ_WRITE(ProjectSettings, alias());
    }
    return firstCall;
}
