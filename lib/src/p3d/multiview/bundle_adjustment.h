#pragma once

#include <set>

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/multiview/bundle_params.h"
#include "p3d/stereo/dense_matching.h"

namespace p3d
{
/**
 * @brief BundleData holds the data that on which the bundle adjustment will
 * operate
 */
struct P3D_API BundleData {
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

    std::vector<Landmark>* landmarks;

    /**
     * @brief checks the validity of the given BA problem
     * @return true if the problem is valid:
     * - R.size() = t.size() = cam.size() = W.rows()/3
     * - W.cols() = X.cols();
     */
    bool isValid() const
    {
        if (landmarks == nullptr) {
            LOG_ERR("Bundle adjustment needs landmarks");
            return false;
        }

        const auto nbCams = R.size();
        if (t.size() != nbCams) return false;
        if (cam.size() != nbCams) return false;
        return true;
    }
};

class P3D_API BundleAdjustment
{
public:
    /*! @brief magic happens here. */
    void run(BundleData& data, BundleParams& params);
    void runPtsOnly(BundleData& data);
};
}  // namespace p3d
