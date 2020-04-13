#pragma once

#include "p3d/core.h"

namespace p3d
{
class ProjectData;

namespace plot
{
void ReprojectionError(const p3d::ProjectData& data, bool* open, int width);
}  // namespace plot

}  // namespace p3d
