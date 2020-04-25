#pragma once

#include <string>
#include <vector>

#include "../../assets/fonts/IconsFontAwesome5.h"

#define P3D_ICON_RUN ICON_FA_CHECK
#define P3D_ICON_RUNALL ICON_FA_CHECK_DOUBLE
#define P3D_ICON_DELETE ICON_FA_TRASH_ALT
#define P3D_ICON_VISIBLE ICON_FA_EYE

std::vector<std::string> loadImagesDialog();
std::string openProjectDialog();
std::string saveProjectDialog();
std::string exportPointCloudDialog();
