#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>

#include "p3d/core.h"
#include "p3d/multiview/landmark.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

#include <map>

namespace p3d
{
enum P3D_API p3dDense_ {
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
    p3dDense_specklesWindowSize,  // Maximum size of smooth disparity regions to consider
                                  // their noise speckles and invalidate. Set it to 0 to
                                  // disable speckle filtering. Otherwise, set it
                                  // somewhere in the 50-200 range.
    p3dDense_specklesRange,       // Maximum disparity variation within each connected
                             // component. If you do speckle filtering, set the parameter
                             // to a positive value, it will be implicitly multiplied
                             // by 16. Normally, 1 or 2 is good enough.
};

class P3D_API DenseMatchingPars : public Serializable<DenseMatchingPars>
{
public:
    enum DenseMatchingMethod_ {
        DenseMatchingMethod_SGBM = cv::StereoSGBM::MODE_SGBM,
        DenseMatchingMethod_HH = cv::StereoSGBM::MODE_HH,
        DenseMatchingMethod_SGBM_3WAY = cv::StereoSGBM::MODE_SGBM_3WAY
    };

    static int initMeta();
    static const char* classNameStatic() { return "DenseMatchingPars"; }

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

class Neighbor
{
public:
    cv::Size imLsize;
    cv::Mat imLrect;
    cv::Mat imRrect;
    cv::Size imRsize{0, 0};
    Mat3f Tl{Mat3f::Identity()};
    Mat3f Tr{Mat3f::Identity()};
    Mat3f Trinv{Mat3f::Identity()};
    Mat3f R{Mat3f::Identity()};
    Vec2f dispRange{0, 0};
    cv::Mat disp;
    cv::Mat confidence;

    inline bool isValid() const { return !disp.empty(); }
    inline bool isInversed() const { return m_isInversed; }

    Neighbor inverse()
    {
        Neighbor ninv;
        ninv.imLsize = imRsize;
        ninv.imRsize = imLsize;
        ninv.imLrect = imRrect.clone();
        ninv.imRrect = imLrect.clone();
        ninv.Tr = Tl;
        ninv.Tl = Tr;
        ninv.Trinv = utils::inverseTransform(ninv.Tr);
        ninv.R = R.transpose();
        ninv.disp = -1.0f * disp;
        ninv.confidence = confidence.clone();
        ninv.valid = valid;
        ninv.m_isInversed = true;
        ninv.dispRange = Vec2f{0, 0};
        return ninv;
    }

private:
    bool m_isInversed{false};
    bool valid{true};
};

struct DenseMatchingUtil {
    static void findDisparity(
        const cv::Mat& imLeftR, const cv::Mat& imRightR, cv::Mat& disparity,
        cv::Mat& confidence, const DenseMatchingPars& denseMatching = DenseMatchingPars(),
        const DenseMatchingPars& denseMatchingRightToLeft = DenseMatchingPars());

    static void filterDisparityBilateral(
        const cv::Mat& disparity, cv::Mat& disparityFiltered,
        const DenseMatchingPars& denseMatching = DenseMatchingPars());

    static void getDispForPlot(const cv::Mat& disparity, cv::Mat& plot,
                               const Vec2f& dispRange = {0, 0});

    static void refineConsistencyCheck(cv::Mat& disparityBase,
                                       const cv::Mat& disparityNeighbor,
                                       float thresPx = 1.0);

    static std::map<int, Observation> findMatch(
        const std::map<int, std::map<int, Neighbor>>& neighbors, int camRef,
        const Vec2f& pt, float confidence = 1.0f);

    static void mergeDisparities(std::map<int, std::map<int, Neighbor>> neighbors,
                                 std::vector<std::map<int, Observation>>& matches);

    constexpr static const float NO_DISPARITY{-10000.0};
};

}  // namespace p3d
