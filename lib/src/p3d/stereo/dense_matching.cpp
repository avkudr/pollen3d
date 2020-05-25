#include "dense_matching.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <queue>
#include <set>

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
        sgbmRL->setMinDisparity(denseMatchingRightToLeft.dispLowerBound);
        sgbmRL->setNumDisparities(16 * denseMatchingRightToLeft.dispUpperBound);
        sgbmRL->compute(imR, imL, disparityMapRL);

        cv::Mat out = 0.0f * disparityMapLR.clone();

        auto wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbmLR);
        wls_filter->setLambda(1.0);
        wls_filter->setSigmaColor(24.0);
        wls_filter->filter(disparityMapLR, imL, out, disparityMapRL);
        confidenceMap = wls_filter->getConfidenceMap();
        cv::Mat confidenceMask;
        confidenceMap.convertTo(confidenceMask, CV_8UC1);
        confidenceMap /= 255.0;
        cv::threshold(confidenceMask, confidenceMask, 0.75 * 255.0, 255.0,
                      cv::THRESH_BINARY);

        cv::Mat blackZoneMask;
        cv::threshold(imL, blackZoneMask, 5, 255, cv::THRESH_BINARY);
        if (blackZoneMask.channels() == 3)
            cvtColor(blackZoneMask, blackZoneMask, cv::COLOR_BGR2GRAY);

        cv::Mat mask = blackZoneMask & confidenceMask;
        if (out.type() != CV_32FC1) out.convertTo(out, CV_32FC1);
        disparity = 0.0f * out + DenseMatchingUtil::NO_DISPARITY;
        out.copyTo(disparity, mask);

        //        cv::imshow("sdqsdqsqs", mask);
        //        cv::waitKeyEx(0);

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
        LOG_ERR("DenseMatchingUtil::getPlot. Wrong type: %i", disparity.type());
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
    LOG_DBG("Disparity plot range: (%0.3f,%0.3f)", Amin / 16.0, Amax / 16.0);
    LOG_DBG("Disparity true range: (%0.3f,%0.3f)", trueMin / 16.0, trueMax / 16.0);

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
                                         std::vector<std::map<int, Observation>> &matches)
{
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

    for (const auto &kv : neighbors) {
        const auto &cRef = kv.first;
        const auto &neighborsRef = kv.second;
        if (neighborsRef.empty()) continue;

        int w = neighborsRef.begin()->second.imLsize.width;
        int h = neighborsRef.begin()->second.imLsize.height;

        for (auto u = 0; u < h; ++u) {
            //#ifdef WITH_OPENMP
            //#pragma omp parallel for
            //#endif
            for (auto v = 0; v < w; ++v) {
                if (visited[cRef].at<uchar>(u, v)) continue;

                std::map<int, Observation> landmark =
                    findMatch(neighbors, cRef, Vec2f(v, u), 1.0);

                for (const auto &[camId, obs] : landmark) {
                    if (visited.count(camId) == 0) continue;
                    visited[camId].at<uchar>(obs.pt(1), obs.pt(0)) = 255;
                }

                if (landmark.size() < 2) continue;

                //#pragma omp critical
                {
                    matches.push_back(landmark);
                }
            }
            std::cout << cRef << " progress: " << u / float(h) << std::endl;

#if 0  // draw
            int winSize = 400;
            for (auto i = 0; i < visited.size(); ++i) {
                std::string name =
                    "(" + std::to_string(i) + ":" + std::to_string(i) + ")";
                cv::Mat plot;
                cv::namedWindow(name, CV_WINDOW_NORMAL);
                cv::imshow(name, visited[i]);
                cv::resizeWindow(name, winSize * 0.9, winSize * 0.9);
                cv::moveWindow(name, (i % 10) * winSize, i / 10 * winSize);
            }
            cv::waitKey(1);
            // cv::destroyAllWindows();
#endif
        }
        std::cout << "Merging disparities: image " << cRef << ", " << matches.size()
                  << " matches" << std::endl;
    }  // ref image
}

void findMatchDFS(const std::map<int, std::map<int, Neighbor>> &neighbors, int camRef,
                  float x, float y, float confidence,
                  std::vector<std::pair<int, Observation>> &landmark,
                  std::vector<std::pair<int, Observation>> &bestLmrk,
                  std::set<int> visited)
{
    if (landmark.empty()) {
        landmark.push_back({camRef, Observation(Vec2(x, y), confidence)});
    } else {
        landmark.push_back(
            {camRef,
             Observation(Vec2(x, y), landmark.back().second.confidence + confidence)});
    }

    visited.insert(camRef);

    if (bestLmrk.empty() ||
        bestLmrk.back().second.confidence < landmark.back().second.confidence)
        bestLmrk = landmark;

    // the complexity of the algorithm is exponential
    // so, when image number is high it is way too slow
    // TODO: better algorithm
    if (visited.size() > 3) return;

    if (neighbors.count(camRef) == 0) return;

    for (const auto &kvref : neighbors.at(camRef)) {
        const int cam2 = kvref.first;
        const Neighbor &n = kvref.second;

        if (!n.isValid()) continue;
        if (visited.count(cam2) > 0) continue;

        Vec3f q = n.Tl * Vec3f(x, y, 1);

        if (q(0) < 0) continue;
        if (q(1) < 0) continue;
        if (q(0) >= n.disp.cols) continue;
        if (q(1) >= n.disp.rows) continue;

        const float &disp = n.disp.at<float>(q(1), q(0));
        if (disp == DenseMatchingUtil::NO_DISPARITY) continue;
        const float &newConfidence =
            (n.confidence.empty()) ? 1.0f : n.confidence.at<float>(q(1), q(0));

        if (newConfidence == 0.0f) continue;

        q(0) -= disp / 16.0;
        q = n.Trinv * q;

        if (q(0) < 0) continue;
        if (q(1) < 0) continue;
        if (q(0) >= n.imRsize.width) continue;
        if (q(1) >= n.imRsize.height) continue;

        findMatchDFS(neighbors, cam2, q(0), q(1), newConfidence, landmark, bestLmrk,
                     visited);
        landmark.pop_back();
    }
}

void findMatchBFS(const std::map<int, std::map<int, Neighbor>> &neighbors, int camRef,
                  float x, float y, float confidence,
                  std::map<int, Observation> &landmark)
{
    std::queue<std::pair<int, Observation>> queue;
    queue.push({camRef, Observation(Vec2(x, y), confidence)});

    landmark.clear();

    while (!queue.empty()) {
        auto m = queue.front();
        queue.pop();

        const auto &camId = m.first;
        const Observation &obs = m.second;
        if (landmark.count(camId) > 0) continue;
        landmark[camId] = obs;

        for (const auto &kvref : neighbors.at(camId)) {
            const int cam2 = kvref.first;
            const Neighbor &n = kvref.second;

            if (!n.isValid()) continue;
            if (landmark.count(cam2) > 0) continue;

            Vec3f q = n.Tl * Vec3f(x, y, 1);

            if (q(0) < 0) continue;
            if (q(1) < 0) continue;
            if (q(0) >= n.disp.cols) continue;
            if (q(1) >= n.disp.rows) continue;

            const float &disp = n.disp.at<float>(q(1), q(0));
            if (disp == DenseMatchingUtil::NO_DISPARITY) continue;
            const float &newConfidence =
                (n.confidence.empty()) ? 1.0f : n.confidence.at<float>(q(1), q(0));

            q(0) -= disp / 16.0;
            q = n.Trinv * q;

            if (q(0) < 0) continue;
            if (q(1) < 0) continue;
            if (q(0) >= n.imRsize.width) continue;
            if (q(1) >= n.imRsize.height) continue;

            queue.push({cam2, Observation(Vec2(q(0), q(1)), newConfidence)});
        }
    }
}

std::map<int, Observation> DenseMatchingUtil::findMatch(
    const std::map<int, std::map<int, Neighbor>> &neighbors, int camRef, const Vec2f &pt,
    float confidence)
{
    std::map<int, Observation> landmark;
    int method = 0;  // the best for now :/
    if (method == 0) {
        std::vector<std::pair<int, Observation>> l, bl;
        std::set<int> visited;
        findMatchDFS(neighbors, camRef, pt(0), pt(1), confidence, l, bl, visited);
        for (auto i = bl.size() - 1; i > 0; --i)
            bl[i].second.confidence -= bl[i - 1].second.confidence;

        for (const auto &m : bl) { landmark[m.first] = m.second; }
        return landmark;
    }

    if (method == 1) {
        findMatchBFS(neighbors, camRef, pt(0), pt(1), confidence, landmark);
        return landmark;
    }
}
