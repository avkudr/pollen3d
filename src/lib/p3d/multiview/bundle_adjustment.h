#pragma once

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/multiview/bundle_params.h"

namespace p3d
{
/**
 * @brief BundleProblem holds the data that on which the bundle adjustment will
 * operate
 */
struct BundleProblem {
    /**
     * @brief vector of rotation vectors (3\times1)
     *
     * @warning pay attention to the fact that each rotation vector defines the
     * rotation between the first image and the i-th image
     */
    std::vector<Vec3> R;

    /**
     * @brief vector of translation vectors (2\times1)
     *
     * @warning pay attention to the fact that each rotation vector defines the
     * rotation between the first image and the i-th image
     */
    std::vector<Vec2> t;

    /**
     * @brief vector of instrinsic parameters (3\times1)
     *
     * parameters are the focal length, the skew, and the aspect ratio
     */
    std::vector<Vec3> cam;

    /// @brief 3D point cloud (4 \times p)
    Mat4X X;

    /// @brief measurement matrix (3c\timesp)
    Mat W;

    /**
     * @brief checks the validity of the given BA problem
     * @return true if the problem is valid:
     * - R.size() = t.size() = cam.size() = W.rows()/3
     * - W.cols() = X.cols();
     */
    bool isValid() const
    {
        int nbPts = X.cols();
        if (W.cols() != nbPts) return false;

        const auto nbCams = R.size();
        if (t.size() != nbCams) return false;
        if (cam.size() != nbCams) return false;
        if (W.rows() / 3 != nbCams) return false;
        return true;
    }
};

class BundleAdjustment {

public:
    /*! @brief magic happens here. */
    void run(BundleProblem & p, BundleParams & params);
};
}  // namespace p3d
