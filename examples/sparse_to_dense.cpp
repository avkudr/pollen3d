#include <iostream>

#include "p3d/logger.h"
#include "p3d/project.h"
#include "p3d/tasks.h"
#include "p3d/utils.h"

#include "p3d/multiview/bundle_adjustment.h"
#include "p3d/stereo/dense_matching.h"
#include "p3d/stereo/rectification.h"

using namespace p3d;

int main()
{
    float threshold = 2.0f;

    std::string file;
    p3d::Project data;
    file = "/home/andrey/Desktop/_datasets/brassica/brassica.yml.gz";
    // file = "/home/andrey/Desktop/_datasets/pot/pot.yml.gz";
    // file = "/home/andrey/Desktop/_datasets/pot1/pot1.yml.gz";
    // file = "/home/andrey/Desktop/_datasets/Ostracode/ostracode.yml.gz";
    // file = "/home/andrey/Desktop/_datasets/ostracode24/ostracode24.yml.gz";
    // file = "/home/andrey/Desktop/_datasets/hayamica/hayamica.yml.gz";
    p3d::openProject(data, file);

    LOG_INFO("Project: %s", data.getName().c_str());

    auto W = data.getMeasurementMatrix();
    auto R = data.getCamerasRotations();

    bool withBA = true;

    // matrix of Neighbor's, size [nbCams x nbCams]

    //    std::vector<std::vector<Neighbor>> neighbors =
    //    std::vector<std::vector<Neighbor>>(
    //        data.nbImages(), std::vector<Neighbor>(data.nbImages()));

    int minNumberCommonPts = 20;
    float minTriangAngle = utils::deg2rad(2.0f);
    float maxTriangAngle = utils::deg2rad(10.0f);

    int imageFrom = 0;
    int imageTo = data.nbImages();  // data.nbImages();

    std::map<int, std::map<int, Neighbor>> neighbors;
    for (int camBaseIdx = imageFrom; camBaseIdx < imageTo - 1; camBaseIdx++) {
#pragma omp parallel for
        for (int i = camBaseIdx + 1; i < imageTo; i++) {
            LOG_INFO("*** Image %i", i);

            Neighbor neighbor;
            Mat Wneighbor;
            utils::findFullSubMeasurementMatrix(W, Wneighbor, {camBaseIdx, i});
            if (Wneighbor.cols() < minNumberCommonPts) continue;

            Mat3 relRotation = R[camBaseIdx] * R[i].transpose();
            double t1, rho, t2;
            utils::EulerZYZtfromR(relRotation, t1, rho, t2);

            LOG_INFO("angle theta1: %0.3f", utils::rad2deg(t1));
            LOG_INFO("angle rho   : %0.3f", utils::rad2deg(rho));
            LOG_INFO("angle theta2: %0.3f", utils::rad2deg(t2));

            if (std::fabs(rho) < minTriangAngle) continue;
            if (std::fabs(rho) > maxTriangAngle) continue;

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
            neighbor.Tl = rectifData.Tl.cast<float>();
            neighbor.Tr = rectifData.Tr.cast<float>();
            neighbor.Trinv = utils::inverseTransform(neighbor.Tr);
            neighbor.imLsize = rectifData.imL.size();
            neighbor.imRsize = rectifData.imR.size();

            Mat3X qL = rectifData.Tl * Wneighbor.middleRows(0, 3);
            Mat3X qR = rectifData.Tr * Wneighbor.middleRows(3, 3);

            // **** disparity

            cv::Mat disp1, disp2;
            cv::Mat plot1, plot2;

            Vec dy = qL.row(1) - qR.row(1);
            Vec dispSparse = qL.row(0) - qR.row(0);

            std::cout << "dy mean, px: " << dy.mean() << std::endl;
            std::cout << "min disparity: " << dispSparse.minCoeff() << std::endl;
            std::cout << "max disparity: " << dispSparse.maxCoeff() << std::endl;

            int minDisparity1 = int(dispSparse.minCoeff()) - 1;
            int nbDisparities1 =
                std::ceil((dispSparse.maxCoeff() - minDisparity1) / 16.0);
            int minDisparity2 = int(-dispSparse.maxCoeff()) - 1;
            int nbDisparities2 =
                std::ceil((-dispSparse.minCoeff() - minDisparity2) / 16.0);

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
            params1.dispBlockSize = params2.dispBlockSize = 5;

            params2 = params1;
            params2.dispLowerBound = minDisparity2;
            params2.dispUpperBound = nbDisparities2;

            Vec2f range1(params1.dispLowerBound, params1.dispUpperBound);
            Vec2f range2(params2.dispLowerBound, params2.dispUpperBound);
            cv::Mat confidence;
            DenseMatchingUtil::findDisparity(neighbor.imLrect, neighbor.imRrect, disp1,
                                             confidence, params1, params2);

            // {
            //        DenseMatchingUtil::getDispForPlot(disp1, plot1, range1);
            //        cv::imshow("disparity1", plot1);
            //        DenseMatchingUtil::getDispForPlot(disp2, plot2, range2);
            //        cv::imshow("disparity2", plot2);

            //        cv::waitKeyEx(0);
            //        cv::destroyAllWindows();
            // }

            // ***** consistency check

            // DenseMatchingUtil::refineConsistencyCheck(disp1, disp2, 2.0);

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
            std::cout << "disparity type: " << disp1.type() << std::endl;
            std::cout << "disparity difference median, px: " << utils::median(diff)
                      << std::endl;
            std::cout << "disparity difference mean, px  : " << utils::mean(diff)
                      << std::endl;

            neighbor.dispRange = range1;
            neighbor.disp = disp1;
            neighbor.confidence = confidence;

#pragma omp critical
            {
                neighbors[camBaseIdx][i] = neighbor;
                neighbors[i][camBaseIdx] = neighbor.inverse();
            }
        }

#if 0  // draw
        int winSize = 150;
        for (const auto& [i, n] : neighbors[camBaseIdx]) {
            std::string name =
                "(" + std::to_string(camBaseIdx) + ":" + std::to_string(i) + ")";
            cv::Mat plot;
            DenseMatchingUtil::getDispForPlot(n.disp, plot, n.dispRange);
            cv::namedWindow(name, CV_WINDOW_NORMAL);
            cv::imshow(name, plot);
            cv::resizeWindow(name, winSize, winSize);
            cv::moveWindow(name, (i % 10) * winSize, i / 10 * winSize);
        }
        cv::waitKeyEx(0);
        cv::destroyAllWindows();
#endif
    }

    // ***** merging disparities

    //    int batchSize = neighbors[0].size() + 1;

    //    int camBaseIdx = 0;
    //    std::vector<std::map<int, Vec2>> matches;
    //    auto size = data.image(camBaseIdx)->cvMat().size();
    //    for (auto x = 0; x < size.width; ++x) {
    //        for (auto y = 0; y < size.height; ++y) {
    //            std::map<int, Vec2> m;
    //            for (int i = 0; i < neighbors[camBaseIdx].size(); i++) {
    //                const auto& n = neighbors[camBaseIdx][i];
    //                if (!n.valid) continue;

    //                Vec3 q = n.Tl * Vec3(x, y, 1);
    //                const float& disp = n.disp.at<float>(q(1), q(0));
    //                if (disp == DenseMatchingUtil::NO_DISPARITY) continue;
    //                Vec3 qnr = Vec3(q(0) - disp / 16.0, q(1), 1);
    //                Vec3 qn = n.Trinv * qnr;
    //                if (qn(0) <= 0) continue;
    //                if (qn(1) <= 0) continue;
    //                if (qn(0) >= n.imRsize.width) continue;
    //                if (qn(1) >= n.imRsize.height) continue;
    //                m.insert({i, qn.topRows(2)});
    //            }
    //            if (m.size() == 0) continue;

    //            m.insert({camBaseIdx, Vec2(x, y)});
    //            matches.push_back(m);
    //        }
    //    }

    std::vector<std::map<int, Observation>> matches;
    DenseMatchingUtil::mergeDisparities(neighbors, matches);

    // ***** triangulate

    std::cout << "Triangulation..." << std::endl;

    std::vector<Landmark> landmarks;
    std::vector<Vec3uc> colors;
    auto Ps = data.getCameraMatrices();

    cv::Mat I1 = data.image(0)->cvMat();
    if (I1.channels() == 1) cv::cvtColor(I1, I1, CV_GRAY2BGR);

    for (auto pt = 0; pt < static_cast<int>(matches.size()); pt++) {
        std::vector<Vec2> xa;
        std::vector<Mat34> Pa;

        bool draw = false && matches[pt].size() > 3;
        // draw = pt % 1000 == 0;

        Landmark l(std::move(matches[pt]));

        for (const auto& [camId, obs] : l.obs) {
            Pa.push_back(Ps[camId]);
            xa.push_back(obs.pt);

            if (draw) {
                cv::Mat I = data.image(camId)->cvMat().clone();
                if (I.channels() == 1) cv::cvtColor(I, I, CV_GRAY2BGR);
                std::string name = "image" + std::to_string(camId);
                cv::drawMarker(I, cv::Point(obs.pt.x(), obs.pt.y()),
                               cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
                cv::imshow(name, I);
                cv::moveWindow(name, camId * I.cols, 100);
            }
        }

        if (draw) {
            cv::waitKeyEx(0);
            cv::destroyAllWindows();
        }

        // if (xa.size() > 2) { void(); }
        Vec4 Xa = utils::triangulate(xa, Pa);
        l.X = Xa.topRows(3);

        float err = utils::reprojectionErrorPt(xa, Pa, Xa);

        const cv::Vec3b& c1 = I1.at<cv::Vec3b>(matches[pt].begin()->second.pt.y(),
                                               matches[pt].begin()->second.pt.x());

        landmarks.push_back(std::move(l));
        colors.emplace_back(Vec3uc(c1.val[0], c1.val[1], c1.val[2]));
    }
    matches.clear();

    std::cout << "X size: " << landmarks.size() << std::endl;

    if (withBA) {
        // ***** construct measurement matrix

        p3d::BundleData bundleData;
        bundleData.landmarks = &landmarks;

        LOG_OK("Bundle adjustement: started...");
        data.getCamerasIntrinsics(&bundleData.cam);
        bundleData.R = data.getCameraRotationsAbsolute();
        bundleData.t = data.getCameraTranslations();

        BundleAdjustment ba;
        ba.runPtsOnly(bundleData);
    }

    Mat3X X;
    LandmarksUtil::toMat3X(landmarks, X);
    data.pointCloudCtnr()["densePcd"].setVertices(X.cast<float>());
    data.pointCloudCtnr()["densePcd"].setColorsRValue(std::move(colors));
    p3d::saveProject(data, "/home/andrey/Desktop/sparse_to_dense.yml.gz");

    return 0;
}
