#pragma once

#include <opencv2/core.hpp>

#include "p3d/data/project_settings.h"
#include "p3d/data/project_data.h"
#include "p3d/core/core.h"
#include "p3d/console_logger.h"
#include "p3d/commands.h"

class ProjectManager{
public:
    static ProjectManager * get() {
        if (!m_instance) {
           m_instance = new ProjectManager();
        }
        return m_instance;
    }

    // ***** Files

    std::vector<std::string> loadImagesDialog();

    std::string openProjectDialog();
    std::string saveProjectDialog();
    void loadImages(ProjectData * list, const std::vector<std::string> & imPaths);

    void saveProject(ProjectData * data, std::string path = "");
    void closeProject(ProjectData * data);
    void openProject(ProjectData * data, std::string path = "");

    // ***** Image

    void extractFeatures(ProjectData & data, std::vector<int> imIds = {});

    // ***** Pairs

    void matchFeatures(ProjectData & data, std::vector<int> imPairsIds = {});
    void findFundamentalMatrix(ProjectData & data, std::vector<int> imPairsIds = {});
    void rectifyImagePairs(ProjectData & data, std::vector<int> imPairsIds = {});
    void findDisparityMap(ProjectData & data, std::vector<int> imPairsIds = {});
    void filterDisparityBilateral(ProjectData & data, std::vector<int> imPairsIds = {});
    void filterDisparitySpeckles(ProjectData & data, std::vector<int> imPairsIds = {});

    // ***** Multiview

    void findMeasurementMatrixFull(ProjectData & data);
    void findMeasurementMatrix(ProjectData & data);
    void autocalibrate(ProjectData & data);
    void triangulate(ProjectData & data);
    void triangulateStereo(ProjectData & data);
    void triangulateDense(ProjectData & data);
    void bundleAdjustment(ProjectData & data);
    void exportPLY(ProjectData & data);

    // ***** Settings

    entt::meta_any getSetting(const p3dSetting & name);
    void setSetting(const p3dSetting & name, const entt::meta_any &value);
    void setSettings(const ProjectSettings & settings){ m_settings = settings; }

    // ***** Data properties

    template <typename T>
    void setProperty(T * instance, const P3D_ID_TYPE &propId, const entt::meta_any & v, bool force = false) {
        if (instance == nullptr) return;
        CommandManager::get()->executeCommand(
                    new CommandSetProperty(instance,propId,v,force));
    }

private:
    ProjectManager() {}
    static ProjectManager * m_instance;
    ProjectSettings m_settings;
};
