#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

#include <opencv2/imgproc.hpp>

namespace p3d
{
P3D_API struct RectificationData {
    // ***** input
    cv::Mat imL;
    cv::Mat imR;
    double angleL;
    double angleR;
    std::vector<Vec2> ptsL;
    std::vector<Vec2> ptsR;

    // ***** output
    cv::Mat imLrect;  /// rectified left image
    cv::Mat imRrect;  /// rectified right image
    Mat3 Tl;          /// rectified transformation for left image
    Mat3 Tr;          /// rectified transformation for right image
    double errorMean{std::numeric_limits<double>::max()};
    double errorStd{std::numeric_limits<double>::max()};
};

P3D_API struct RectificationUtil {
    static void rectify(RectificationData& data);
};
}  // namespace p3d
