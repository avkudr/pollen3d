#include "dense_matching.h"

#include <opencv2/imgproc.hpp>

using namespace p3d;

int dummyDenseMatching_ = DenseMatchingPars::initMeta();

int DenseMatchingPars::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: DenseMatching");

        SERIALIZED_ADD_READ_WRITE(DenseMatchingPars);

        entt::meta<DenseMatchingPars>()
            .data<&DenseMatchingPars::dispMethod>(P3D_ID_TYPE(p3dDense_DispMethod))
            .data<&DenseMatchingPars::dispLowerBound>(
                P3D_ID_TYPE(p3dDense_DispLowerBound))
            .data<&DenseMatchingPars::dispUpperBound>(
                P3D_ID_TYPE(p3dDense_DispUpperBound))
            .data<&DenseMatchingPars::dispBlockSize>(P3D_ID_TYPE(p3dDense_DispBlockSize))
            .data<&DenseMatchingPars::dispFilterNewValue>(
                P3D_ID_TYPE(p3dDense_DispFilterNewValue))
            .data<&DenseMatchingPars::dispFilterMaxSpeckleSize>(
                P3D_ID_TYPE(p3dDense_DispFilterMaxSpeckleSize))
            .data<&DenseMatchingPars::dispFilterMaxDiff>(
                P3D_ID_TYPE(p3dDense_DispFilterMaxDiff))
            .data<&DenseMatchingPars::bilateralD>(P3D_ID_TYPE(p3dDense_BilateralD))
            .data<&DenseMatchingPars::bilateralSigmaColor>(
                P3D_ID_TYPE(p3dDense_BilateralSigmaColor))
            .data<&DenseMatchingPars::bilateralSigmaSpace>(
                P3D_ID_TYPE(p3dDense_BilateralSigmaSpace));

        firstCall = false;
    }
    return 0;
}

void DenseMatchingUtil::findDisparity(const cv::Mat &imLeftR,
                                      const cv::Mat &imRightR,
                                      cv::Mat &disparity,
                                      const DenseMatchingPars &denseMatching)
{
    const auto &lowerBound = denseMatching.dispLowerBound;
    const auto &upperBound = 16 * denseMatching.dispUpperBound;
    const auto &blockSize = denseMatching.dispBlockSize;

    try {
        cv::Ptr<cv::StereoSGBM> sgbm =
            cv::StereoSGBM::create(lowerBound,
                                   upperBound,  // number of disparities
                                   blockSize);

        sgbm->setMode(denseMatching.dispMethod);

        int cn = imLeftR.channels();
        sgbm->setP1(8 * cn * blockSize * blockSize);
        sgbm->setP2(32 * cn * blockSize * blockSize);

        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(1);

        cv::Mat disparityMap;
        sgbm->compute(imLeftR, imRightR, disparityMap);

        disparity = disparityMap.clone();

        if (disparity.type() != CV_32FC1) disparity.convertTo(disparity, CV_32FC1);

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
    try {
        cv::bilateralFilter(m, disparityFiltered, d, sc, ss);
    } catch (...) {
        disparityFiltered = cv::Mat();
    }
}

void DenseMatchingUtil::getDispForPlot(const cv::Mat &disparity, cv::Mat &plot,
                                       const Vec2f &dispRange)
{
    plot = cv::Mat();
    if (disparity.type() != CV_32F) {
        LOG_ERR("DenseMatchingUtil::getPlot. Wrong type");
        return;
    }
    float Amin, Amax;
    float trueMin, trueMax;
    trueMin = *std::min_element(disparity.begin<float>(), disparity.end<float>());
    trueMax = *std::max_element(disparity.begin<float>(), disparity.end<float>());
    if (dispRange(0) == dispRange(1)) {
        Amin = *std::min_element(disparity.begin<float>(), disparity.end<float>());
        Amax = *std::max_element(disparity.begin<float>(), disparity.end<float>());
    } else {
        Amin = 16.0f * dispRange(0) + -16.0f;
        Amax = Amin + dispRange(1) * 16.0f * 16.0f;
    }
    LOG_INFO("Disparity plot range: (%0.3f,%0.3f)", Amin / 16.0, Amax / 16.0);
    LOG_INFO("Disparity true range: (%0.3f,%0.3f)", trueMin / 16.0, trueMax / 16.0);

    cv::Mat inRange;
    cv::inRange(disparity, Amin, Amax, inRange);
    plot = disparity * 0 + Amin;
    disparity.copyTo(plot, inRange);

    cv::Mat A_scaled = (plot - Amin) / (Amax - Amin);
    A_scaled.convertTo(plot, CV_8UC1, 255.0, 0);
    applyColorMap(plot, plot, cv::COLORMAP_HOT);
}

void DenseMatchingUtil::refineConsistencyCheck(cv::Mat &disparityBase,
                                               const cv::Mat &disparityNeighbor,
                                               float thresPx)
{
    p3d_Assert(thresPx > 0.0f);
    p3d_Assert(!disparityBase.empty());
    p3d_Assert(!disparityNeighbor.empty());
    p3d_Assert(disparityBase.type() == CV_32FC1);
    p3d_Assert(disparityNeighbor.type() == CV_32FC1);
    p3d_Assert(disparityBase.size() == disparityNeighbor.size());

    cv::Mat dispN = disparityNeighbor.clone();

    cv::Mat kernel = cv::Mat::ones(3, 3, CV_32F) / 9.0f;
    cv::filter2D(disparityNeighbor, dispN, -1, kernel);

    float thres = 16.0f * thresPx;
    float *p1;
    for (int i = 0; i < disparityBase.rows; ++i) {
        p1 = disparityBase.ptr<float>(i);
        const auto p2 = disparityNeighbor.ptr<float>(i);

        for (int j = 0; j < disparityBase.cols; ++j) {
            int j2 = j + p1[j] / 16.0f;
            if (j2 >= disparityNeighbor.cols) continue;

            const float &d1 = p1[j];
            const float &d2 = p2[j];
            if (std::fabs(d1 + d2) > thres)
                p1[j] = NO_DISPARITY;
            else
                p1[j] = (d1 - d2) / 2.0f;
        }
    }
}
