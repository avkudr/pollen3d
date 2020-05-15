#include <iostream>

#include "p3d/project.h"
#include "p3d/logger.h"
#include "p3d/tasks.h"
#include "p3d/utils.h"

#include "p3d/multiview/bundle_adjustment.h"
#include "p3d/stereo/dense_matching.h"
#include "p3d/stereo/rectification.h"

using namespace p3d;

int main()
{
    float threshold = 2.0f;

    p3d::Project data;
    p3d::openProject(data, "/home/andrey/Desktop/_datasets/brassica/brassica.yml.gz");
    // p3d::openProject(data, "/home/andrey/Desktop/_datasets/pot/pot.yml.gz");
    // p3d::openProject(data, "/home/andrey/Desktop/_datasets/pot1/pot1.yml.gz");

    LOG_INFO("Project: %s", data.getName().c_str());

    auto W = data.getMeasurementMatrix();
    auto R = data.getCamerasRotations();

    int camBaseIdx = 0;

    struct Neighbor {
        int imLidx;
        int imRidx;
        cv::Mat imLrect;
        cv::Mat imRrect;
        cv::Size imRsize;
        Mat3 Tl;
        Mat3 Tr;
        Mat3 Trinv;
        Mat3 R;
        Vec2f dispRange;

        cv::Mat disp;
    };

    std::vector<Neighbor> neighbors;
    int minNumberCommonPts = 20;
    float minTriangAngle = utils::deg2rad(2.0f);
    float maxTriangAngle = utils::deg2rad(20.0f);
    for (int i = 0; i < data.nbImages(); i++) {
        if (i == camBaseIdx) continue;

        LOG_INFO("*** Image %i", i);

        Neighbor neighbor;
        neighbor.imLidx = camBaseIdx;
        neighbor.imRidx = i;

        Mat Wneighbor;
        utils::findFullSubMeasurementMatrix(W, Wneighbor, {camBaseIdx, i});
        if (Wneighbor.cols() < minNumberCommonPts) continue;

        Mat3 relRotation = R[camBaseIdx] * R[i].transpose();
        double t1, rho, t2;
        utils::EulerZYZtfromR(relRotation, t1, rho, t2);

        LOG_INFO("angle theta1: %0.3f", utils::rad2deg(t1));
        LOG_INFO("angle rho   : %0.3f", utils::rad2deg(rho));
        LOG_INFO("angle theta2: %0.3f", utils::rad2deg(t2));

        if (rho < minTriangAngle) continue;
        if (rho > maxTriangAngle) continue;

        RectificationData rectifData;
        rectifData.imL = data.image(camBaseIdx)->cvMat();
        rectifData.imR = data.image(i)->cvMat();
        rectifData.angleL = t1;
        rectifData.angleR = t2;
        utils::convert(Mat2X(Wneighbor.middleRows(0, 2)), rectifData.ptsL);
        utils::convert(Mat2X(Wneighbor.middleRows(3, 2)), rectifData.ptsR);

        RectificationUtil::rectify(rectifData);

        LOG_INFO("rectification error, mean: %0.3f", rectifData.errorMean);
        LOG_INFO("rectification error, std : %0.3f", rectifData.errorStd);

        neighbor.imLrect = rectifData.imLrect.clone();
        neighbor.imRrect = rectifData.imRrect.clone();
        neighbor.Tl = rectifData.Tl;
        neighbor.Tr = rectifData.Tr;
        neighbor.Trinv = utils::inverseTransform(neighbor.Tr);
        neighbor.imRsize = rectifData.imR.size();

        Mat3X qL = neighbor.Tl * Wneighbor.middleRows(0, 3);
        Mat3X qR = neighbor.Tr * Wneighbor.middleRows(3, 3);

        // **** disparity

        cv::Mat disp1, disp2;
        cv::Mat plot1, plot2;

        Vec dy = qL.row(1) - qR.row(1);
        Vec dispSparse = qL.row(0) - qR.row(0);

        std::cout << "dy mean, px: " << dy.mean() << std::endl;
        std::cout << "min disparity: " << dispSparse.minCoeff() << std::endl;
        std::cout << "max disparity: " << dispSparse.maxCoeff() << std::endl;

        int minDisparity1 = int(dispSparse.minCoeff()) - 1;
        int nbDisparities1 = std::ceil((dispSparse.maxCoeff() - minDisparity1) / 16.0);
        int minDisparity2 = int(-dispSparse.maxCoeff()) - 1;
        int nbDisparities2 = std::ceil((-dispSparse.minCoeff() - minDisparity2) / 16.0);

        p3d_Assert(nbDisparities1 > 0);
        p3d_Assert(nbDisparities2 > 0);

        std::cout << "b) min disparity: " << minDisparity1 << std::endl;
        std::cout << "b) max disparity: " << nbDisparities1 << std::endl;
        std::cout << "2) min disparity: " << minDisparity2 << std::endl;
        std::cout << "2) max disparity: " << nbDisparities2 << std::endl;

        // ***** estimate disparity

        DenseMatchingPars params1;
        DenseMatchingPars params2;

        params1.dispLowerBound = minDisparity1;
        params1.dispUpperBound = nbDisparities1;
        params2.dispLowerBound = minDisparity2;
        params2.dispUpperBound = nbDisparities2;
        params1.dispBlockSize = params2.dispBlockSize = 5;

        Vec2f range1(params1.dispLowerBound, params1.dispUpperBound);
        Vec2f range2(params2.dispLowerBound, params2.dispUpperBound);
        DenseMatchingUtil::findDisparity(neighbor.imLrect, neighbor.imRrect, disp1,
                                         params1);
        DenseMatchingUtil::findDisparity(neighbor.imRrect, neighbor.imLrect, disp2,
                                         params2);

        // {
        //        DenseMatchingUtil::getDispForPlot(disp1, plot1, range1);
        //        cv::imshow("disparity1", plot1);
        //        DenseMatchingUtil::getDispForPlot(disp2, plot2, range2);
        //        cv::imshow("disparity2", plot2);

        //        cv::waitKeyEx(0);
        //        cv::destroyAllWindows();
        // }

        // ***** consistency check

        DenseMatchingUtil::refineConsistencyCheck(disp1, disp2, 2.0);

        // ***** compare new disparities with old ones

        std::vector<float> diff;
        for (int i = 0; i < qL.cols(); ++i) {
            int x = qL(0, i);
            int y = qL(1, i);
            float dsparse = dispSparse(i);

            float d = disp1.at<float>(y, x) / 16.0;
            // std::cout << dsparse << " : " << d << std::endl;
            diff.push_back(dsparse - d);
        }
        std::cout << "disparity difference median, px: " << utils::median(diff)
                  << std::endl;
        std::cout << "disparity difference mean, px  : " << utils::mean(diff)
                  << std::endl;

        neighbor.dispRange = range1;
        neighbor.disp = disp1;
        neighbors.push_back(neighbor);
    }

    for (const auto& n : neighbors) {
        std::string name = "disparity" + std::to_string(n.imRidx);
        cv::Mat plot;
        DenseMatchingUtil::getDispForPlot(n.disp, plot, n.dispRange);
        cv::imshow(name, plot);
        cv::moveWindow(name, n.imRidx * plot.cols, 50);
    }
    cv::waitKeyEx(0);
    cv::destroyAllWindows();

    // ***** merging disparities

    int batchSize = neighbors.size() + 1;

    std::vector<std::map<int, Vec2>> matches;
    auto size = data.image(camBaseIdx)->cvMat().size();
    for (auto x = 0; x < size.width; ++x) {
        for (auto y = 0; y < size.height; ++y) {
            std::map<int, Vec2> m;
            for (const auto& n : neighbors) {
                Vec3 q = n.Tl * Vec3(x, y, 1);
                const float& disp = n.disp.at<float>(q(1), q(0));
                if (disp == DenseMatchingUtil::NO_DISPARITY) continue;
                Vec3 qnr = Vec3(q(0) - disp / 16.0, q(1), 1);
                Vec3 qn = n.Trinv * qnr;
                if (qn(0) <= 0) continue;
                if (qn(1) <= 0) continue;
                if (qn(0) >= n.imRsize.width) continue;
                if (qn(1) >= n.imRsize.height) continue;
                m.insert({n.imRidx, qn.topRows(2)});
            }
            if (m.size() == 0) continue;

            m.insert({camBaseIdx, Vec2(x, y)});
            matches.push_back(m);
        }
    }

    // ***** construct measurement matrix

    Mat Wdense;
    Wdense.setZero(3 * data.nbImages(), matches.size());

    for (auto pt = 0; pt < matches.size(); pt++) {
        for (const auto& kv : matches[pt]) {
            const auto& c = kv.first;
            const auto& x = kv.second;

            Wdense(3 * c + 0, pt) = x.x();
            Wdense(3 * c + 1, pt) = x.y();
            Wdense(3 * c + 2, pt) = 1.0;
        }
    }

    // ***** triangulate

    Mat4X X;
    X.setZero(4, matches.size());
    auto Ps = data.getCameraMatrices();

    Mat3Xf colors;
    colors.setZero(3, matches.size());

    cv::Mat I1 = data.image(0)->cvMat();
    if (I1.channels() == 1) cv::cvtColor(I1, I1, CV_GRAY2BGR);

    for (auto pt = 0; pt < matches.size(); pt++) {
        std::vector<Vec2> xa;
        std::vector<Mat34> Pa;

        bool draw = false;
        // draw = pt % 1000 == 0;

        for (const auto& kv : matches[pt]) {
            Pa.push_back(Ps[kv.first]);
            xa.push_back(kv.second);

            if (draw) {
                cv::Mat I = data.image(kv.first)->cvMat().clone();
                if (I.channels() == 1) cv::cvtColor(I, I, CV_GRAY2BGR);
                std::string name = "image" + std::to_string(kv.first);
                cv::drawMarker(I, cv::Point(kv.second.x(), kv.second.y()),
                               cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
                cv::imshow(name, I);
                cv::moveWindow(name, kv.first * I.cols, 100);
            }
        }

        if (draw) {
            cv::waitKeyEx(0);
            cv::destroyAllWindows();
        }

        // if (xa.size() > 2) { void(); }
        Vec4 Xa = utils::triangulate(xa, Pa);
        X.col(pt) = Xa;

        float err = utils::reprojectionErrorPt(xa, Pa, Xa);

        const cv::Vec3b& c1 = I1.at<cv::Vec3b>(matches[pt][0].y(), matches[pt][0].x());
        colors.col(pt) = Vec3f(c1.val[0] / 255.f, c1.val[1] / 255.f, c1.val[2] / 255.f);

        // colors.col(pt) = err > 10.0 ? Vec3f(1.0, 0.0, 0.0) : Vec3f(1.0, 1.0, 1.0);
    }

    p3d::BundleData bundleData;
    bundleData.X = X;
    bundleData.W = Wdense;

    LOG_OK("Bundle adjustement: started...");
    data.getCamerasIntrinsics(&bundleData.cam);
    bundleData.R = data.getCameraRotationsAbsolute();
    bundleData.t = data.getCameraTranslations();

    {
        BundleParams params(data.nbImages());
        params.setConstAll();
        params.setVaryingPts();
        BundleAdjustment ba;
        ba.run(bundleData, params);
    }
    X = bundleData.X;

    data.pointCloudCtnr()["denseLala"].setVertices(X.topRows(3).cast<float>());
    data.pointCloudCtnr()["denseLala"].setColors(colors);
    p3d::saveProject(data, "sparse_to_dense.yml.gz");

    return 0;
}
