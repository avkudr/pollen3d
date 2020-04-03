#pragma once

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/multiview/bundle_params.h"

namespace p3d
{
struct BundleProblem {
    std::vector<Vec3> R;
    std::vector<Vec2> t;
    std::vector<Vec3> cam;

    Mat4X X;
    Mat W;
};

class BundleAdjustment {

public:
    void run(BundleProblem & p, BundleParams & params);
};
}  // namespace p3d
