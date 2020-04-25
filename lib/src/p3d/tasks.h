#pragma once

#include <opencv2/core.hpp>

#include "p3d/core.h"
#include "p3d/data/project_settings.h"
#include "p3d/project.h"


namespace p3d
{

namespace task {

int total();
int progress();
float progressPercent();
std::string name();
void reset();

}
// ***** Undo manager

void P3D_API undo();
void P3D_API redo();
void P3D_API mergeNextCommand();

// ***** Files

void P3D_API loadImages(Project &data, const std::vector<std::string> &imPaths);

void P3D_API saveProject(Project *data, std::string path = "");
void P3D_API closeProject(Project *data);
void P3D_API openProject(Project *data, std::string path = "");

// ***** Image

bool P3D_API extractFeatures(Project &data, std::vector<int> imIds = {});

// ***** Pairs

bool P3D_API matchFeatures(Project &data, std::vector<int> imPairsIds = {});
bool P3D_API findFundamentalMatrix(Project &data, std::vector<int> imPairsIds = {});
bool P3D_API rectifyImagePairs(Project &data, std::vector<int> imPairsIds = {});
bool P3D_API findDisparityMap(Project &data, std::vector<int> imPairsIds = {});
void P3D_API filterDisparityBilateral(Project &data, std::vector<int> imPairsIds = {});
void P3D_API filterDisparitySpeckles(Project &data, std::vector<int> imPairsIds = {});

// ***** Multiview

void P3D_API findMeasurementMatrixFull(Project &data);
void P3D_API findMeasurementMatrix(Project &data);
void P3D_API autocalibrate(Project &data);
void P3D_API triangulateSparse(Project &data);
void P3D_API triangulateDenseStereo(Project &data, std::vector<int> imPairsIds = {});
void P3D_API triangulateDenseMultiview(Project &data);
void P3D_API triangulateDenseDev(Project &data);
void P3D_API bundleAdjustment(Project &data);
void P3D_API exportPLY(const Project &data, const std::string &label,
                       const std::string &filepath);
void P3D_API deletePointCloud(Project &data, const char *lbl);

// ***** Settings

entt::meta_any getSetting(Project &project, const p3dSetting &name);
void setSetting(Project &project, const p3dSetting &name, const entt::meta_any &value);

// ***** Misc

void P3D_API setImageProperty(Project &             data,
                              const P3D_ID_TYPE &   propId,
                              const entt::meta_any &value,
                              std::vector<int>      imPairsIds = {});

void P3D_API copyImageProperty(Project &data, const P3D_ID_TYPE &propId, int from,
                               const std::vector<int> &to = {});

void P3D_API setImagePairProperty(Project &data, const P3D_ID_TYPE &propId,
                                  const entt::meta_any &value,
                                  std::vector<int> imPairsIds = {});
void P3D_API copyImagePairProperty(Project &data, const P3D_ID_TYPE &propId, int from,
                                   const std::vector<int> &to = {});

}  // namespace p3d
