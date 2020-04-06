#include "dense_matching.h"

#include <opencv2/imgproc.hpp>

using namespace p3d;

int dummyDenseMatching_ = DenseMatchingPars::initMeta();

int DenseMatchingPars::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: DenseMatching" << std::endl;
        entt::meta<DenseMatchingPars>()
            .type("DenseMatching"_hs)
            .data<&DenseMatchingPars::dispMethod>(
                P3D_ID_TYPE(p3dDense_DispMethod))
            .data<&DenseMatchingPars::dispLowerBound>(
                P3D_ID_TYPE(p3dDense_DispLowerBound))
            .data<&DenseMatchingPars::dispUpperBound>(
                P3D_ID_TYPE(p3dDense_DispUpperBound))
            .data<&DenseMatchingPars::dispBlockSize>(
                P3D_ID_TYPE(p3dDense_DispBlockSize))
            .data<&DenseMatchingPars::dispFilterNewValue>(
                P3D_ID_TYPE(p3dDense_DispFilterNewValue))
            .data<&DenseMatchingPars::dispFilterMaxSpeckleSize>(
                P3D_ID_TYPE(p3dDense_DispFilterMaxSpeckleSize))
            .data<&DenseMatchingPars::dispFilterMaxDiff>(
                P3D_ID_TYPE(p3dDense_DispFilterMaxDiff))
            .data<&DenseMatchingPars::bilateralD>(
                P3D_ID_TYPE(p3dDense_BilateralD))
            .data<&DenseMatchingPars::bilateralSigmaColor>(
                P3D_ID_TYPE(p3dDense_BilateralSigmaColor))
            .data<&DenseMatchingPars::bilateralSigmaSpace>(
                P3D_ID_TYPE(p3dDense_BilateralSigmaSpace));

        SERIALIZED_ADD_READ_WRITE(DenseMatchingPars, "DenseMatching"_hs);
        firstCall = false;
    }
    return 0;
}

void DenseMatchingUtil::findDisparity(const cv::Mat &imLeftR,
                                      const cv::Mat &imRightR,
                                      cv::Mat &disparity,
                                      const DenseMatchingPars &denseMatching)
{
    const auto &lowerBound = 16 * denseMatching.dispLowerBound;
    const auto &upperBound = 16 * denseMatching.dispUpperBound;
    const auto &blockSize = denseMatching.dispBlockSize;

    cv::Ptr<cv::StereoSGBM> sgbm =
        cv::StereoSGBM::create(lowerBound,
                               upperBound,  // number of disparities
                               blockSize);

    sgbm->setMode(denseMatching.dispMethod);

    int cn = imLeftR.channels();
    sgbm->setP1(8 * cn * blockSize * blockSize);
    sgbm->setP2(32 * cn * blockSize * blockSize);

    try {
        cv::Mat disparityMap;
        sgbm->compute(imLeftR, imRightR, disparityMap);

        if (disparityMap.type() != CV_32F)
            disparityMap.convertTo(disparityMap, CV_32F);

        disparity = disparityMap.clone();
    } catch (...) {
        disparity = cv::Mat();
    }
}

void DenseMatchingUtil::filterDisparityBilateral(
    const cv::Mat &disparity, cv::Mat &disparityFiltered,
    const DenseMatchingPars &denseMatching)
{
    cv::Mat m = disparity.clone();
    if (m.type() != CV_32F) m.convertTo(m, CV_32F);

    auto d = denseMatching.bilateralD;
    auto sc = denseMatching.bilateralSigmaColor;
    auto ss = denseMatching.bilateralSigmaSpace;

    disparityFiltered = cv::Mat();
    cv::bilateralFilter(m, disparityFiltered, d, sc, ss);
}

void DenseMatchingUtil::getDispForPlot(const cv::Mat &disparity, cv::Mat &plot)
{
    plot = cv::Mat();
    if (disparity.type() != CV_32F) {
        LOG_ERR("DenseMatchingUtil::getPlot. Wrong type");
        return;
    }
    float Amin =
        *std::min_element(disparity.begin<float>(), disparity.end<float>());
    float Amax =
        *std::max_element(disparity.begin<float>(), disparity.end<float>());
    cv::Mat A_scaled = (disparity - Amin) / (Amax - Amin);
    A_scaled.convertTo(plot, CV_8UC1, 255.0, 0);
    applyColorMap(plot, plot, cv::COLORMAP_HOT);
}
