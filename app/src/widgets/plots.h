#pragma once

#include "p3d/core.h"

namespace p3d
{
class Project;

namespace plot
{
void ReprojectionError(const p3d::Project& data, bool* open, int width);
}  // namespace plot

}  // namespace p3d
