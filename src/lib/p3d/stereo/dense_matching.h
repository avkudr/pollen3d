#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>

#include "p3d/core.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

namespace p3d
{
enum p3dDense_ {
    p3dDense_DispMap = 0,
    p3dDense_DispMethod,
    p3dDense_DispLowerBound,
    p3dDense_DispUpperBound,
    p3dDense_DispBlockSize,

    p3dDense_DispFilterNewValue,        // The disparity value used to paint-off the
                                        // speckles
    p3dDense_DispFilterMaxSpeckleSize,  // The maximum speckle size to consider
                                        // it a speckle. Larger blobs are not
                                        // affected by the algorithm
    p3dDense_DispFilterMaxDiff,         // Maximum difference between neighbor
                                        // disparity pixels to put them into the same
                                        // blob. Note that since StereoBM,
    // StereoSGBM and may be other algorithms return a fixed-point disparity
    // map, where disparity values are multiplied by 16, this scale factor
    // should be taken into account when specifying this parameter value.

    p3dDense_BilateralD,           // The disparity value used to paint-off the speckles
    p3dDense_BilateralSigmaColor,  // The maximum speckle size to consider it a
                                   // speckle. Larger blobs are not affected by
                                   // the algorithm
    p3dDense_BilateralSigmaSpace,  // Maximum difference between neighbor
                                   // disparity pixels to put them into the same
                                   // blob. Note that since StereoBM,
    // StereoSGBM and may be other algorithms return a fixed-point disparity
    // map, where disparity values are multiplied by 16, this scale factor
    // should be taken into account when specifying this parameter value.

};

class DenseMatchingPars : public Serializable<DenseMatchingPars>
{
public:
    enum DenseMatchingMethod_ {
        DenseMatchingMethod_SGBM = cv::StereoSGBM::MODE_SGBM,
        DenseMatchingMethod_HH = cv::StereoSGBM::MODE_HH,
        DenseMatchingMethod_SGBM_3WAY = cv::StereoSGBM::MODE_SGBM_3WAY
    };

    static int initMeta();

    cv::Mat dispMap;

    int dispMethod{DenseMatchingMethod_SGBM};
    int dispLowerBound{-1};
    int dispUpperBound{2};
    int dispBlockSize{9};

    int dispFilterNewValue{0};
    int dispFilterMaxSpeckleSize{260};
    int dispFilterMaxDiff{10};

    int bilateralD{9};
    int bilateralSigmaColor{180};
    int bilateralSigmaSpace{180};
};

struct DenseMatchingUtil {
    static void findDisparity(
        const cv::Mat& imLeftR, const cv::Mat& imRightR, cv::Mat& disparity,
        const DenseMatchingPars& denseMatching = DenseMatchingPars());

    static void filterDisparityBilateral(
        const cv::Mat& disparity, cv::Mat& disparityFiltered,
        const DenseMatchingPars& denseMatching = DenseMatchingPars());

    static void getDispForPlot(const cv::Mat& disparity, cv::Mat& plot);
};

}  // namespace p3d
