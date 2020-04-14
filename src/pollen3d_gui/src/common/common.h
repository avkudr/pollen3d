#pragma once

#include <string>
#include <vector>

#include "../../assets/fonts/IconsFontAwesome5.h"

#include "tinyfiledialogs/tinyfiledialogs.h"

#include "p3d/core.h"
#include "p3d/utils.h"

#define P3D_ICON_RUN ICON_FA_CHECK
#define P3D_ICON_RUNALL ICON_FA_CHECK_DOUBLE
#define P3D_ICON_DELETE ICON_FA_TRASH_ALT
#define P3D_ICON_VISIBLE ICON_FA_EYE

static std::vector<std::string> loadImagesDialog()
{
    std::vector<std::string> out;
    char const *lTheOpenFileName;
    char const *lFilterPatterns[] = {"*.jpg", "*.jpeg", "*.png", "*.tif",
                                     "*.tiff"};

    lTheOpenFileName = tinyfd_openFileDialog("Load images...", "", 5,
                                             lFilterPatterns, nullptr, 1);
    if (!lTheOpenFileName) return out;

    std::string allFiles(lTheOpenFileName);
    out = p3d::utils::split(lTheOpenFileName, "|");
    return out;
}

static std::string openProjectDialog()
{
    char const *lTheOpenFileName;
    std::string ext = "*" + std::string(P3D_PROJECT_EXTENSION);
    char const *lFilterPatterns[] = {ext.c_str()};

    lTheOpenFileName = tinyfd_openFileDialog("Load images...", "", 1,
                                             lFilterPatterns, nullptr, 0);
    if (!lTheOpenFileName) return "";

    std::string projectFile(lTheOpenFileName);
    return projectFile;
}

static std::string saveProjectDialog()
{
    char const *saveFilePath;
    std::string ext = "*" + std::string(P3D_PROJECT_EXTENSION);
    char const *saveFilePattern[] = {ext.c_str()};

    saveFilePath = tinyfd_saveFileDialog("Save project as...", "",
                                         1,  // nb files to save
                                         saveFilePattern, "Pollen3D project");

    if (!saveFilePath) return "";

    std::string out(saveFilePath);
    if (!p3d::utils::endsWith(out, P3D_PROJECT_EXTENSION))
        out += P3D_PROJECT_EXTENSION;
    return out;
}

static std::string exportPointCloudDialog()
{
    char const *saveFilePath;
    std::string ext = "*" + std::string(".ply");
    char const *saveFilePattern[] = {ext.c_str()};

    saveFilePath = tinyfd_saveFileDialog("Export point cloud...", "point_could",
                                         1,  // nb files to save
                                         saveFilePattern, "Point cloud");

    if (!saveFilePath) return "";

    std::string out(saveFilePath);
    if (!p3d::utils::endsWith(out, ".ply")) out += ".ply";
    return out;
}
