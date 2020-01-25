#pragma once

#include <opencv2/opencv.hpp>

#include "p3d/data/project_settings.h"
#include "p3d/data/project_data.h"
#include "p3d/core/core.h"
#include "p3d/console_logger.h"

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
    void openProject(ProjectData * data, std::string path = "");

    // ***** Image

    void extractFeatures(ProjectData & imList, std::vector<int> imIds = {});
    void matchFeatures(ProjectData & imList, std::vector<int> imPairsIds = {});

    meta::any getSetting(const p3dSetting & name);

    void setSetting(const p3dSetting & name, meta::any value);

private:
    ProjectManager() {}
    static ProjectManager * m_instance;
    ProjectSettings settings;

};
