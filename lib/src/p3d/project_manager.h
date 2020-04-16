#pragma once

#include <opencv2/core.hpp>

#include "p3d/commands.h"
#include "p3d/core.h"
#include "p3d/data/project_data.h"
#include "p3d/data/project_settings.h"
#include "p3d/logger.h"

namespace p3d
{
class ProjectManager
{
public:
    static ProjectManager *get() {
        static ProjectManager *instance = nullptr;
        if (!instance) instance = new ProjectManager();
        return instance;
    }
    // ***** Files

    void loadImages(ProjectData *list, const std::vector<std::string> &imPaths);

    void saveProject(ProjectData *data, std::string path = "");
    void closeProject(ProjectData *data);
    void openProject(ProjectData *data, std::string path = "");

    // ***** Image

    void extractFeatures(ProjectData &data, std::vector<int> imIds = {});

    // ***** Pairs

    bool matchFeatures(ProjectData &data, std::vector<int> imPairsIds = {});
    bool findFundamentalMatrix(ProjectData &data, std::vector<int> imPairsIds = {});
    bool rectifyImagePairs(ProjectData &data, std::vector<int> imPairsIds = {});
    bool findDisparityMap(ProjectData &data, std::vector<int> imPairsIds = {});
    void filterDisparityBilateral(ProjectData &data, std::vector<int> imPairsIds = {});
    void filterDisparitySpeckles(ProjectData &data, std::vector<int> imPairsIds = {});

    // ***** Multiview

    void findMeasurementMatrixFull(ProjectData &data);
    void findMeasurementMatrix(ProjectData &data);
    void autocalibrate(ProjectData &data);
    void triangulateSparse(ProjectData &data);
    void triangulateDenseStereo(ProjectData &data, std::vector<int> imPairsIds = {});
    void triangulateDenseMultiview(ProjectData &data);
    void triangulateDenseDev(ProjectData &data);
    void bundleAdjustment(ProjectData &data);
    void exportPLY(const ProjectData &data, const std::string &label,
                   const std::string &filepath);
    void deletePointCloud(ProjectData &data, const char *lbl);

    // ***** Settings

    const ProjectSettings &settings() const { return m_settings; }
    entt::meta_any getSetting(const p3dSetting &name);
    void setSetting(const p3dSetting &name, const entt::meta_any &value);
    void setSettings(const ProjectSettings &settings) { m_settings = settings; }

    // ***** Misc

    void setImagePairProperty(ProjectData &data, const P3D_ID_TYPE &propId,
                              const entt::meta_any &value,
                              std::vector<int> imPairsIds = {});
    void copyImagePairProperty(ProjectData &data, const P3D_ID_TYPE &propId,
                               int from, const std::vector<int> &to = {});

    // ***** Data properties

    template <typename T>
    void setProperty(T *instance, const P3D_ID_TYPE &propId, const entt::meta_any &v, bool force = false)
    {
        if (instance == nullptr) {
            LOG_ERR("Can't set property of null");
            return;
        }
        CommandManager::get()->executeCommand(
            new CommandSetProperty(instance, propId, v, force));
    }

private:
    ProjectManager() {}
    ProjectSettings m_settings;
};
}  // namespace p3d
