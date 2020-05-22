#include "dense_matching.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

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

void DenseMatchingUtil::findDisparity(const cv::Mat &imLeftR, const cv::Mat &imRightR,
                                      cv::Mat &disparity, cv::Mat &confidenceMap,
                                      const DenseMatchingPars &denseMatching,
                                      const DenseMatchingPars &denseMatchingRightToLeft)
{
    cv::Mat imL = imLeftR.clone();
    cv::Mat imR = imRightR.clone();

    if (imL.channels() == 1) cvtColor(imL, imL, cv::COLOR_BGR2GRAY);
    if (imR.channels() == 1) cvtColor(imR, imR, cv::COLOR_BGR2GRAY);

    const auto &minDisparity = denseMatching.dispLowerBound;
    const auto &numDisparities = 16 * denseMatching.dispUpperBound;
    const auto &blockSize = denseMatching.dispBlockSize;

    try {
        cv::Ptr<cv::StereoSGBM> sgbmLR =
            cv::StereoSGBM::create(minDisparity,
                                   numDisparities,  // number of disparities
                                   blockSize);

        sgbmLR->setMode(denseMatching.dispMethod);

        int cn = imL.channels();
        sgbmLR->setP1(8 * cn * blockSize * blockSize);
        sgbmLR->setP2(32 * cn * blockSize * blockSize);

        cv::Mat disparityMapLR;
        sgbmLR->compute(imL, imR, disparityMapLR);

        cv::Mat disparityMapRL;
        cv::Ptr<cv::StereoSGBM> sgbmRL(sgbmLR);
        sgbmLR->setMinDisparity(denseMatchingRightToLeft.dispLowerBound);
        sgbmLR->setNumDisparities(16 * denseMatchingRightToLeft.dispUpperBound);
        sgbmLR->compute(imR, imL, disparityMapRL);

        cv::Mat out = 0.0f * disparityMapLR.clone();

        auto wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbmLR);
        wls_filter->setLambda(1.0);
        wls_filter->setSigmaColor(24.0);
        wls_filter->filter(disparityMapLR, imL, out, disparityMapRL);
        confidenceMap = wls_filter->getConfidenceMap();
        cv::Mat confidenceMask;
        confidenceMap.convertTo(confidenceMask, CV_8UC1);
        cv::threshold(confidenceMask, confidenceMask, 0.75 * 255.0, 255.0,
                      cv::THRESH_BINARY);

        if (out.type() != CV_32FC1) out.convertTo(out, CV_32FC1);
        disparity = 0.0f * out + DenseMatchingUtil::NO_DISPARITY;
        out.copyTo(disparity, confidenceMask);

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

void DenseMatchingUtil::mergeDisparities(std::map<int, std::map<int, Neighbor>> neighbors,
                                         std::vector<std::map<int, Vec2>> &matches)
{
    /*
    matches.clear();
    if (neighbors.empty()) return;

    std::map<int, cv::Mat> visited;
    for (const auto &kvref : neighbors) {
        int cRef = kvref.first;
        const auto &neighborsRef = kvref.second;
        if (neighborsRef.empty()) continue;

        int w = neighborsRef.begin()->second.imLsize.width;
        int h = neighborsRef.begin()->second.imLsize.height;
        visited[cRef] = cv::Mat::zeros(h, w, CV_8UC1);
    }

    for (const auto &kvref : neighbors) {
        int cRef = kvref.first;
        const auto &neighborsRef = kvref.second;
        if (neighborsRef.empty()) continue;

        int w = neighborsRef.begin()->second.imLsize.width;
        int h = neighborsRef.begin()->second.imLsize.height;

        for (auto u = 0; u < h; ++u) {
            for (auto v = 0; v < w; ++v) {
                if (visited[cRef].at<uchar>(u, v)) continue;

                std::map<int, std::vector<MatchCandidate>> bearingCand;
                bearingCand = findMatch(neighbors, cRef, Vec2f(v, u), 1.0);

#if 1  // print bearing
                std::cout << "{" << std::endl;
                for (const auto &m : bearingCand) {
                    std::cout << " - " << m.first << ": ";
                    for (const auto &pt : m.second) {
                        std::cout << "(" << pt.pt(0) << ":" << pt.confidence << ") ";
                    }
                    std::cout << std::endl;
                }
                std::cout << "}" << std::endl;
#endif
                // merge bearingCand in bearing
                std::map<int, Vec2> bearing;
                for (const auto &e : bearingCand) { bearing[e.first] = e.second[0].pt; }
                matches.push_back(bearing);

                for (const auto &e : bearing) {
                    if (visited.count(e.first) == 0) continue;
                    visited[e.first].at<uchar>(e.second(1), e.second(0)) = 255;
                }

#if 1  // draw visited
                for (const auto &e : visited) {
                    auto im = e.first;
                    std::string name = "image" + std::to_string(im);
                    cv::namedWindow(name, cv::WINDOW_NORMAL);
                    cv::imshow(name, e.second);
                    cv::resizeWindow(name, 300, 300);
                    cv::moveWindow(name, 100 + im * 300, 100);
                }
                cv::waitKeyEx(0);
#endif
            }
        }

    }  // ref image
    */
}

void findMatchImpl(const std::map<int, std::map<int, Neighbor>> &neighbors, int camRef,
                   float u, float v, float confidence,
                   std::vector<MatchCandidate> &landmark,
                   std::vector<MatchCandidate> &bestLmrk, std::set<int> visited)
{
    if (landmark.empty()) {
        landmark.emplace_back(MatchCandidate(Vec2(v, u), camRef, confidence));
    } else {
        landmark.emplace_back(
            MatchCandidate(Vec2(v, u), camRef, landmark.back().confidence + confidence));
    }

    //    std::cout << "---------- " << std::endl;
    //    for (const auto &l : landmark) { l.print(); }

    visited.insert(camRef);

    if (bestLmrk.empty() || bestLmrk.back().confidence < landmark.back().confidence)
        bestLmrk = landmark;

    //    for (const auto &v : visited) std::cout << v << " ";
    //    std::cout << std::endl;

    if (neighbors.count(camRef) == 0) return;

    for (const auto &kvref : neighbors.at(camRef)) {
        const int cam2 = kvref.first;
        const Neighbor &n = kvref.second;

        if (!n.isValid()) continue;

        Vec3f q = n.Tl * Vec3f(v, u, 1);
        const float &disp = n.disp.at<float>(q(1), q(0));
        if (disp == DenseMatchingUtil::NO_DISPARITY) continue;
        q(0) -= disp / 16.0;
        const Vec3f qn = n.Trinv * q;

        if (qn(0) < 0) continue;
        if (qn(1) < 0) continue;
        if (qn(0) >= n.imRsize.width) continue;
        if (qn(1) >= n.imRsize.height) continue;

        const float &newConfidence =
            (n.confidence.empty()) ? 1.0f : n.confidence.at<float>(q(1), q(0));

        findMatchImpl(neighbors, cam2, qn(1), qn(0), newConfidence, landmark, bestLmrk,
                      visited);
        landmark.pop_back();
    }
}

std::map<int, MatchCandidate> DenseMatchingUtil::findMatch(
    const std::map<int, std::map<int, Neighbor>> &neighbors, int camRef, const Vec2f &pt,
    float confidence)
{
    std::map<int, MatchCandidate> landmark;
    std::set<int> visited;
    std::vector<MatchCandidate> l, bl;
    findMatchImpl(neighbors, camRef, pt(1), pt(0), confidence, l, bl, visited);

    if (bl.empty()) return landmark;

    landmark.insert({bl[0].camId, bl[0]});
    for (auto i = bl.size() - 1; i > 0; --i) {
        bl[i].confidence -= bl[i - 1].confidence;
        landmark.insert({bl[i].camId, bl[i]});
    }
    return landmark;
};
