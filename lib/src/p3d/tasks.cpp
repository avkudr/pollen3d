#include "tasks.h"

#include <omp.h>

#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <atomic>
#include <map>
#include <mutex>
#include <vector>

#include "p3d/data/image.h"
#include "p3d/logger.h"
#include "p3d/multiview/autocalib.h"
#include "p3d/multiview/bundle_adjustment.h"
#include "p3d/stereo/fundmat.h"
#include "p3d/stereo/rectification.h"
#include "p3d/utils.h"
#include "p3d/export/openmvs.h"

#include "p3d/command_manager.h"

#define P3D_PROJECT_EXTENSION ".yml.gz"

using namespace p3d;

namespace p3d::task {
std::atomic<int> total_{0};
std::atomic<int> progress_{0};
std::string name_{""};
std::mutex mutex;

void reset()
{
    total_ = 0;
    progress_ = 0;
    std::lock_guard<std::mutex> l(mutex);
    name_ = "";
}

int total() { return total_; }

int progress() { return progress_; }

std::string name()
{
    std::lock_guard<std::mutex> l(mutex);
    return name_;
}

float progressPercent()
{
    if (total_ == 0) return 1.0f;
    return float(progress_) / float(total_);
}

void setName(const std::string &str)
{
    std::lock_guard<std::mutex> l(mutex);
    name_ = str;
}

}  // namespace p3d::task

void p3d::autocalibrateBatch(Project &data)
{
    LOG_OK("Autocalibration: started...");
    // **** prerequisites

    p3d::cmder::undoOff();
    p3d::findFundamentalMatrix(data);
    p3d::findMeasurementMatrix(data);
    p3d::cmder::undoOn();

    const auto &W = data.getMeasurementMatrix();
    if (W.rows() == 0 || W.cols() == 0) {
        p3d_Error("measurement matrix is empty.\nno matches?");
        return;
    }

    // **** parameters

    const int batchSize = data.getAutocalibPars().batchSize;
    const int minNbMatchesInBatch = data.getAutocalibPars().batchMinNbMatches;
    const bool withBA = data.getAutocalibPars().withBA;

    // **** start

    p3d::task::setName("Autocalibration");
    p3d::task::total_ = data.nbImages();
    p3d::task::progress_ = 0;

    Mat4X X;
    X.setZero(4,W.cols());

    std::set<int> calibrated;
    auto batches = utils::generateBatches(data.nbImages(), batchSize);
    const int nbCamsTotal = data.nbImages();

    int batchIdx = 0;

    for (auto batchId = 0; batchId < batches.size(); ++batchId) {
        const auto &batch = batches[batchId];
        for (auto i : batch) calibrated.insert(i);

        const int nbCams = batch.size();

        // **** get full measurement matrix for selected batch

        std::vector<int> selectedPts;
        Mat Wfsub;
        utils::findFullSubMeasurementMatrix(W, Wfsub, batch, &selectedPts);

        if (Wfsub.cols() < minNbMatchesInBatch) {
            std::string err = "not enough matches in batch\n images ";
            for (auto i : batch) err += std::to_string(i) + " ";
            p3d_Error(err);
        }

        // **** get slopes

        Mat2X slopes;
        slopes.setZero(2, nbCams);
        for (auto i = 0; i < nbCams - 1; ++i) {
            const auto &imL = batch[i];
            const auto &imR = batch[i + 1];
            if (!data.hasPair(imL, imR)) {
                LOG_WARN("batch has a non-existing pair: %i, %i", batch[i], batch[i + 1]);
                continue;
            }

            const auto &n = data.imagePairs()[imL][imR];
            slopes(0, i + 1) = n.getTheta1();  // from fundamental matrix
            slopes(1, i + 1) = n.getTheta2();
        }

        // **** autocalibration

        AutoCalibrator autocalib(nbCams);
        autocalib.setMaxTime(60);
        autocalib.setMeasurementMatrix(Wfsub);
        autocalib.setSlopeAngles(slopes);

        autocalib.run();

        std::vector<Vec2> ta = autocalib.getTranslations();
        std::vector<Mat3> Ra = autocalib.getRotations();

        Vec2 t0 = data.getCameraTranslations()[batch[0]];
        Mat3 R0 = data.getCamerasRotations()[batch[0]];

        bool batchesNeedAlign = (t0.norm() > 1e-2) && batchId != 0;
        if (batchesNeedAlign) {
            Vec2 t0a = ta[0];
            for (auto i = 0; i < Ra.size(); ++i) {
                const Mat3 R = Ra[i] * R0;
                ta[i] = R.block<2, 2>(0, 0) * (t0 - t0a) + R.topRightCorner(2, 1) + ta[i];
                Ra[i] = R;
            }
        }

        for (auto n = 0; n < nbCams; ++n) {
            const auto i = batch[n];
            AffineCamera c;
            data.image(i)->setCamera(c);
            data.image(i)->setTranslation(ta[n]);
            data.image(i)->setRotationFromMatrix(Ra[n]);
        }

        // ***** triangulate missing

        std::vector<int> calibVec;
        for (const auto &d : calibrated) calibVec.push_back(d);

        auto Ps = data.getCameraMatrices();
        for (auto pt = 0; pt < W.cols(); pt++) {
            // if (X(3,pt) == 1) continue;
            std::vector<Vec2> xa;
            std::vector<Mat34> Pa;
            for (const auto c : calibVec) {
                if (W(3 * c + 2, pt) != 1.0) continue;
                Pa.push_back(Ps[c]);
                xa.push_back(W.block(3 * c, pt, 2, 1));
            }
            if (xa.size() < 2) continue;
            Vec4 Xa = utils::triangulate(xa, Pa);
            Xa /= Xa(3);
            X.col(pt) = Xa;
        }

        // ***** bundle selected cams and selected points

        if (withBA) {
            LOG_OK("Bundle adjustement: started...");

            BundleData bundleData;
            std::vector<Landmark> landmarks;
            LandmarksUtil::from3DPtsMeasMat(X.topRows(3), W, landmarks);
            bundleData.landmarks = &landmarks;

            data.getCamerasIntrinsics(&bundleData.cam);
            bundleData.R = data.getCameraRotationsAbsolute();
            bundleData.t = data.getCameraTranslations();

            {
                BundleParams params(nbCamsTotal);
                params.setUsedCams(calibVec);
                params.setConstAll();
                params.setVaryingPts();
                BundleAdjustment ba;
                ba.run(bundleData, params);
            }

            {
                BundleParams params(nbCamsTotal);
                params.setConstAll();
                params.setUsedCams(calibVec);
                params.setVaryingAllCams(p3dBundleParam_R);
                params.setVaryingAllCams(p3dBundleParam_t);
                params.setConstAllParams({0});
                BundleAdjustment ba;
                ba.run(bundleData, params);
            }

            {
                BundleParams params(nbCamsTotal);
                params.setConstAll();
                params.setUsedCams(calibVec);
                params.setVaryingPts();
                BundleAdjustment ba;
                ba.run(bundleData, params);
            }

            data.setCamerasIntrinsics(bundleData.cam);
            data.setCameraRotationsAbsolute(bundleData.R);
            data.setCameraTranslations(bundleData.t);

            LandmarksUtil::toMat4X(landmarks, X);

            LOG_OK("Bundle adjustement: done");
        }

        {
            std::cout << "nb triangulated: " << X.bottomRows(1).sum() << "/" << X.cols()
                      << std::endl;
            Vec e = utils::reprojectionError(W, data.getCameraMatricesMat(), X, calibVec);
            std::cout << "mean reproj: " << e.mean() << std::endl;
            std::cout << "reproj error < 0.5px: "
                      << 100.0f * (e.array() < 0.5).count() / float(e.size()) << " %"
                      << std::endl;
            std::cout << "reproj error < 1.0px: "
                      << 100.0f * (e.array() < 1.0).count() / float(e.size()) << " %"
                      << std::endl;
        }

        batchIdx++;
        p3d::task::progress_ = calibVec.size();
    }

    Mat3Xf pts3D = X.topRows(3).cast<float>();
    p3d::cmder::executeCommand(
        new CommandPointCloudAdd(&data.pointCloudCtnr(), "sparse", pts3D));

    p3d::task::reset();
}

void p3d::loadImages(Project &list, const std::vector<std::string> &imPaths)
{
    p3d::task::name_ = "Loading images";

    if (imPaths.empty()) {
        LOG_ERR("Image list is empty");
        return;
    }

    const size_t nbIm = imPaths.size();
    list.clear();

    std::vector<Image> imgs;
    LOG_INFO("Loading %i images...", nbIm);
    for (size_t i = 0; i < nbIm; ++i) {
        Image m(imPaths[i]);
        if (m.isValid()) imgs.emplace_back(m);
    }

    if (imgs.empty()) {
        LOG_ERR("Image loading failed");
        return;
    }

    LOG_OK("Loaded %i/%i", imgs.size(), imPaths.size());
    LOG_WARN("No Undo functionnality");

    list.setImageList(imgs);
    p3d::task::reset();
}

void p3d::saveProject(Project &data, std::string path)
{
    p3d::task::name_ = "Saving project";
    if (path == "") {
        LOG_ERR("Can't save the project to empty path");
        return;
    }

    data.setProjectPath(path);
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "Project"
           << "{";
        data.write(fs);
        fs << "}";
        LOG_OK("Project %s succesfully saved", path.c_str());
        fs.release();
    } else {
        LOG_ERR("Can't write to %s", path.c_str());
    }

    p3d::task::reset();
}

void p3d::closeProject(Project &data)
{
    data = Project();
}

void p3d::openProject(Project &data, std::string path)
{
    p3d::task::name_ = "Opening project";

    if (path == "") return;
    if (!utils::endsWith(path, P3D_PROJECT_EXTENSION)) return;

    p3d::closeProject(data);

    LOG_OK("Loading %s", path.c_str());

    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            data.read(fs["Project"]);
            LOG_OK("Done");
            fs.fs.release();
        } else {
            LOG_ERR("Can't write to %s", path.c_str());
        }
    } catch (const cv::Exception &e) {
        LOG_ERR("cvErr:%s", e.what());
    } catch (...) {
        LOG_ERR("Unknown error");
    }

    p3d::task::reset();
}

bool p3d::extractFeatures(Project &data, std::vector<int> imIds)
{
    if (data.nbImages() == 0) return false;
    if (imIds.empty())
        for (int i = 0; i < data.nbImages(); ++i) imIds.push_back(i);

    auto nbImgs = static_cast<int>(imIds.size());

    p3d::task::setName("Feature extraction");
    p3d::task::total_ = nbImgs;
    p3d::task::progress_ = 0;

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(nbImgs, utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (auto i = 0; i < nbImgs; i++) {
        Image *im = data.image(imIds[i]);
        if (!im) continue;
        if (im->cvMat().empty()) {
            LOG_ERR("features cannot be extracted: no image loaded");
            continue;
        }

        try
        {
            std::vector<cv::KeyPoint> kpts;
            cv::Mat                   desc;
            auto                      pars = im->getFeatExtractionPars();

            FeatExtractionUtil::extract(im->cvMat(), pars, kpts, desc);

            if (kpts.empty())
            {
                LOG_OK("%s: no features", im->name().c_str());
                continue;
            }

            im->setKeyPoints(kpts);
            im->setDescriptors(desc);
            LOG_OK("%s: extracted %i features", im->name().c_str(), int(kpts.size()));

#pragma omp critical
            {
                for (auto pair = 0; pair < data.nbImagePairs(); pair++) {
                    if (!data.imagePair(pair)) continue;
                    if (data.imagePair(pair)->imL() != i) continue;
                    if (!data.imagePair(pair)->hasMatches()) continue;

                    data.imagePair(pair)->deleteMatches();
                    LOG_WARN("Pair %i: matches are no longer valid", pair);
                }
                p3d::task::progress_++;
            }
        } catch (...)
        {
            LOG_ERR("Image %i, feature extraction failed", i);
        }
    }

    LOG_WARN("no undo functionnality");
    p3d::task::reset();
    return true;
}

bool p3d::matchFeatures(Project &data)
{
    if (data.nbImages() == 0) return false;

    const auto nbImgs = data.nbImages();

    p3d::task::name_ = "Matching";
    p3d::task::total_ = nbImgs;
    p3d::task::progress_ = 0;

    CommandGroup *groupCmd = new CommandGroup();

    for (int imIdxL = 0; imIdxL < nbImgs - 1; ++imIdxL) {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
        for (int imIdxR = imIdxL + 1; imIdxR < nbImgs; ++imIdxR) {
            auto imL = data.image(imIdxL);
            auto imR = data.image(imIdxR);
            if (imL == nullptr || imR == nullptr) continue;
            if (!imL->hasFeatures() || !imR->hasFeatures()) {
                LOG_ERR("Features must be extracted before matching");
                continue;
            }

            MatchingPars pars;
            pars.method = data.settings()->matchingMethod;
            pars.filterCoeff = data.settings()->matchingFilterCoeff;

            const auto &_descriptorsLeftImage = imL->getDescriptors();
            const auto &_descriptorsRightImage = imR->getDescriptors();

            std::vector<Match> matchesPair;
            MatchingUtil::match(_descriptorsLeftImage, _descriptorsRightImage, pars,
                                matchesPair);

            if (matchesPair.size() < 10) {
                // LOG_WARN("Pair %i-%i has no matches", imIdxL, imIdxR);
                continue;
            }

#pragma omp critical
            {
                //                Neighbor *n = imL->neighbor(imIdxR);
                //                if (n == nullptr) n = imL->createNeighbor(imIdxL,
                //                imIdxR);

                Neighbor n(imIdxL, imIdxR);
                n.setMatches(matchesPair);
                data.imagePairs()[imIdxL][imIdxR] = n;

                LOG_INFO("Pair %i-%i: %i matches", imIdxL, imIdxR, matchesPair.size());
            }
        }

        p3d::task::progress_ = imIdxL;
    }

    //    auto cmd = new CommandSetProperty{imPair, p3dImagePair_matches, matchesPair};

    //    //#pragma omp critical
    //    {
    //        p3d::task::progress_++;
    //        groupCmd->add(cmd);
    //        LOG_OK("Pair %i, matched %i features", imPairsIds[i], matchesPair.size());
    //        if (!cmd->isValid())
    //            LOG_WARN("Pair %i, matches didn't change", imPairsIds[i]);
    //    }

    LOG_DBG_WARN("matching: no undo!");
    if (groupCmd->empty()) {
        delete groupCmd;
        return false;
    } else
        p3d::cmder::executeCommand(groupCmd);

    p3d::task::reset();
    return true;
}

bool p3d::findFundamentalMatrix(Project &data)
{
    const int nbPairs = data.nbImagePairs();
    if (nbPairs == 0) return false;

    p3d::task::name_ = "Epipolar geometry";
    p3d::task::total_ = nbPairs;
    p3d::task::progress_ = 0;

    CommandGroup *groupCmd = new CommandGroup();

    auto &imagePairs = data.imagePairs();

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(imagePairs.size()), utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (auto i = 0; i < static_cast<int>(imagePairs.size()); ++i) {
        auto it = imagePairs.begin();
        std::advance(it, i);

        const auto imL = (*it).first;
        auto &pairs = (*it).second;
        for (auto &kvR : pairs) {
            const auto imR = kvR.first;
            auto &neighbor = kvR.second;
            std::vector<Vec2> ptsL, ptsR;
            data.getPairwiseMatches(imL, imR, ptsL, ptsR);
            if (ptsL.empty() || ptsR.empty()) {
                LOG_OK("Pair %i-%i has no matches", imL, imR);
                continue;
            }

            Mat3 F = FundMatUtil::findAffineCeres(ptsL, ptsR);
            auto angles = FundMatUtil::slopAngles(F);
            double theta1 = angles.first;
            double theta2 = angles.second;

            auto cmd1 = new CommandSetProperty{&neighbor, p3dNeighbor_fundMat, F};
            auto cmd2 = new CommandSetProperty{&neighbor, p3dNeighbor_theta1, theta1};
            auto cmd3 = new CommandSetProperty{&neighbor, p3dNeighbor_theta2, theta2};

#pragma omp critical
            {
                p3d::task::progress_++;

                groupCmd->add(cmd1);
                groupCmd->add(cmd2);
                groupCmd->add(cmd3);

                if (cmd1->isValid())
                    LOG_OK("Pair %i-%i, estimated F (ceres)", imL, imR);
                else
                    LOG_WARN("Pair %i-%i, F didn't change", imL, imR);
            }
        }
    }

    if (groupCmd->empty()) {
        delete groupCmd;
        return false;
    } else
        p3d::cmder::executeCommand(groupCmd);

    p3d::task::reset();
    return true;
}

bool p3d::rectifyImagePairs(Project &data, std::vector<int> imPairsIds)
{
    LOG_INFO("p3d::rectifyImagePairs");

    if (data.nbImagePairs() == 0) {
        LOG_ERR("No image pair to rectify");
        return false;
    }

    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) {
            LOG_INFO("rectifyImagePairs: %i", i);
            imPairsIds.push_back(i);
        }

    CommandGroup *groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(
        std::min(int(imPairsIds.size()), utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];

        auto imPair = data.imagePair(i);
        if (!imPair) continue;

        auto imL = data.image(imPair->imL());
        auto imR = data.image(imPair->imR());
        if (!imL || !imR) continue;

        double angleL = imPair->getTheta1();
        double angleR = imPair->getTheta2();

        if (utils::floatEq(angleL, 0.0) && utils::floatEq(angleR, 0.0)) {
            LOG_ERR("Pair %i, needs slope angles (epipolar geometry)", i);
            continue;
        }

        std::vector<Vec2> ptsL, ptsR;
        data.getPairwiseMatches(idx, idx + 1, ptsL, ptsR);

        if (imL->cvMat().empty()) {
            LOG_ERR("Pair %i, no left image", i);
            continue;
        }
        if (imR->cvMat().empty()) {
            LOG_ERR("Pair %i, no right image", i);
            continue;
        }

        RectificationData rectif;
        rectif.imL = imL->cvMat();
        rectif.imR = imR->cvMat();
        rectif.angleL = angleL;
        rectif.angleR = angleR;
        rectif.ptsL = ptsL;
        rectif.ptsR = ptsR;

        RectificationUtil::rectify(rectif);

        //        imPair->setRectifyingTransformL(rectif.Tl);
        //        imPair->setRectifyingTransformR(rectif.Tr);
        //        imPair->setRectifiedImageL(rectif.imLrect);
        //        imPair->setRectifiedImageR(rectif.imRrect);

        auto cmd1 = new CommandSetProperty{imPair, p3dImagePair_rectifyingTransformLeft,
                                           rectif.Tl};
        auto cmd2 = new CommandSetProperty{imPair, p3dImagePair_rectifyingTransformRight,
                                           rectif.Tr};
        auto cmd3 =
            new CommandSetPropertyCV{imPair, &ImagePair::setRectifiedImageL,
                                     &ImagePair::getRectifiedImageL, rectif.imLrect};
        auto cmd4 =
            new CommandSetPropertyCV{imPair, &ImagePair::setRectifiedImageR,
                                     &ImagePair::getRectifiedImageR, rectif.imRrect};

#pragma omp critical
        {
            LOG_OK("Pair %i, rectification... done", i);
            LOG_INFO("- error mean: %.3f px", rectif.errorMean);
            LOG_INFO("- error std: %.3f px", rectif.errorStd);
            groupCmd->add(cmd1);
            groupCmd->add(cmd2);
            groupCmd->add(cmd3);
            groupCmd->add(cmd4);
        }
    }

    if (groupCmd->empty()) {
        delete groupCmd;
        return false;
    } else
        p3d::cmder::executeCommand(groupCmd);

    return true;
}

bool p3d::findDisparityMap(Project &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return false;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    p3d::task::name_ = "Dense matching";
    p3d::task::total_ = imPairsIds.size();
    p3d::task::progress_ = 0;

    CommandGroup *groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(
        std::min(int(imPairsIds.size()), utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];
        if (!data.imagePair(i)) continue;
        if (!data.imagePair(i)->isRectified()) continue;

        const auto &imLeftR = data.imagePair(i)->getRectifiedImageL();
        const auto &imRightR = data.imagePair(i)->getRectifiedImageR();

        cv::Mat disparityMap, confidenceMap;
        DenseMatchingUtil::findDisparity(imLeftR, imRightR, disparityMap, confidenceMap,
                                         data.imagePair(i)->getDenseMatchingPars());

        if (disparityMap.empty()) {
            LOG_ERR("Pair %i: disparity estimation failed", i);
            continue;
        }

        auto cmd =
            new CommandSetPropertyCV{data.imagePair(i), &ImagePair::setDisparityMap,
                                     &ImagePair::getDisparityMap, disparityMap};

#pragma omp critical
        {
            p3d::task::progress_++;
            groupCmd->add(cmd);
            LOG_OK("Pair %i, estimated disparity", i);
        }
    }

    if (groupCmd->empty()) {
        delete groupCmd;
        return false;
    } else
        p3d::cmder::executeCommand(groupCmd);

    p3d::task::reset();
    return true;
}

void p3d::filterDisparityBilateral(Project &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    CommandGroup *groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(
        std::min(int(imPairsIds.size()), utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];
        auto imPair = data.imagePair(i);
        if (!imPair) continue;
        if (!imPair->hasDisparityMap()) continue;

        cv::Mat mFiltered;
        DenseMatchingUtil::filterDisparityBilateral(
            imPair->getDisparityMap(), mFiltered,
            imPair->getDenseMatchingPars());

        if (mFiltered.empty()) {
            LOG_OK("Pair %i, bilateral filter failed", i);
            continue;
        }

        auto cmd = new CommandSetPropertyCV{imPair, &ImagePair::setDisparityMap,
                                            &ImagePair::getDisparityMap, mFiltered};
#pragma omp critical
        {
            groupCmd->add(cmd);
            LOG_OK("Pair %i, bilateral filter applied", i);
        }
    }

    if (groupCmd->empty())
        delete groupCmd;
    else
        p3d::cmder::executeCommand(groupCmd);
}

void p3d::filterDisparitySpeckles(Project &data, std::vector<int> imPairsIds)
{
    LOG_ERR("Not implemented yet");
}

void p3d::findMeasurementMatrix(Project &data)
{
    Mat Wfull;
    auto nbIm = data.nbImages();
    auto nbPairs = data.nbImagePairs();
    if (nbPairs == 0) p3d_Error("project has no image pairs");

    // map<pait<imL,imR>, std::vector<pair<uint32_t, uint32_t>>>
    PairWiseMatchesMap matchesMaps;

    for (const auto &kvL : data.imagePairs()) {
        for (const auto &kvR : kvL.second) {
            const auto &n = kvR.second;
            const auto imL = kvL.first;
            const auto imR = kvR.first;
            auto m = n.getMatchesAsVecOfPairs();
            matchesMaps[{imL, imR}] = std::move(m);
        }
    }

    Tracks tracks;
    MatchingUtil::matchesMapsToTable(std::move(matchesMaps), tracks);

    auto nbTracks = tracks.size();
    Wfull.setZero(3 * nbIm, nbTracks);

    int col = 0;
    for (const auto &kv : tracks) {
        for (const auto &kvImagePoint : kv.second) {
            const auto &c = kvImagePoint.first;
            const auto &pId = kvImagePoint.second;

            const auto &image = data.getImage(c);
            const auto &pt = image.getKeyPoints()[pId].pt;
            Wfull.block(3 * c, col, 3, 1) << pt.x, pt.y, 1.0;
        }
        ++col;
    }

    // data.setMeasurementMatrixFull(Wfull);
    p3d::cmder::executeCommand(
        new CommandSetProperty{&data, P3D_ID_TYPE(p3dProject_measMat), Wfull});
    LOG_OK("measurement matrix: %ix%i", Wfull.rows(), Wfull.cols());
}

void p3d::triangulateSparse(Project &data)
{
    LOG_DBG("MAKE BUTTON DISABLED IF NO CALIB + NO FULL W");

    auto Wf = data.getMeasurementMatrix();
    if (Wf.rows() == 0 || Wf.cols() == 0) {
        LOG_ERR("No full measurement matrix");
        return;
    }

    auto Ps = data.getCameraMatrices();
    if (Ps.size() == 0) {
        LOG_ERR("No camera matrices");
        return;
    }

    auto nbCams = Ps.size();
    auto nbPts = Wf.cols();
    Mat3Xf pts3D;
    pts3D.setZero(3, nbPts);

    for (auto p = 0; p < nbPts; ++p) {
        std::vector<Vec2> x;
        std::vector<Mat34> P;
        for (auto c = 0; c < nbCams; c++) {
            if (Wf(3 * c + 2, p) == 1.0) {  // there is a point
                x.emplace_back(Wf.block(3 * c, p, 2, 1));
                P.emplace_back(Ps[c]);
            }
        }
        auto pt = utils::triangulate(x, P);
        pt /= pt(3);
        pts3D.col(p) = pt.topRows(3).cast<float>();
    }

    p3d::cmder::executeCommand(
        new CommandPointCloudAdd(&data.pointCloudCtnr(), "sparse", pts3D));

    LOG_OK("Triangulated %i points",
           data.pointCloudCtnr()["sparse"].nbPoints());
}

void p3d::triangulateDenseStereo(Project &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    CommandGroup *groupCmd = new CommandGroup();

    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto pairIdx = imPairsIds[idx];
        auto imPair = data.imagePair(pairIdx);
        if (!imPair) {
            LOG_ERR("pair %i error", pairIdx);
            continue;
        }
        if (!imPair->hasDisparityMap()) {
            LOG_ERR("pair %i has no disparity map", pairIdx);
            continue;
        }

        auto rho = imPair->getRho();
        if (utils::floatEq(rho, 0.0)) {
            LOG_ERR("pair %i has no rho angle (or equals zero)", pairIdx);
            continue;
        }

        float cosRho = std::cos(rho);
        float sinRhoInv = 1.0 / std::sin(rho);

        auto dispValues = data.imagePair(pairIdx)->getDisparityMap().clone();
        if (dispValues.type() != CV_32F) {
            LOG_ERR("pair %i, disparity map type must be CV_32F", pairIdx);
            continue;
        }
        dispValues = dispValues / 16.0;

        std::vector<Vec3f> pts3D;
        std::vector<Vec3uc> colors;

        cv::Mat Irl = data.imagePair(pairIdx)->getRectifiedImageL();
        cv::Mat Irr = data.imagePair(pairIdx)->getRectifiedImageR();
        if (Irl.type() != CV_8UC3)
            LOG_DBG("Rectified image has wrong type: %i", Irl.type());
        if (Irr.type() != CV_8UC3)
            LOG_DBG("Rectified image has wrong type: %i", Irr.type());

        Mat3 Tlinv =
            utils::inverseTransform(data.imagePair(pairIdx)->getRectifyingTransformL());
        Mat3 Trinv =
            utils::inverseTransform(data.imagePair(pairIdx)->getRectifyingTransformR());
        Mat34 Pl = data.getCameraMatrices()[imPair->imL()];
        Mat34 Pr = data.getCameraMatrices()[imPair->imR()];
        std::vector<Mat34> P = {Pl, Pr};

        p3d::task::name_ = "Trianulation, pair " + std::to_string(pairIdx);
        p3d::task::total_ = dispValues.cols;
        p3d::task::progress_ = 0;

        const auto blackPx = cv::Vec3b(0, 0, 0);

        for (int u1 = 0; u1 < dispValues.cols; ++u1) {
#ifdef WITH_OPENMP
        omp_set_num_threads(utils::nbAvailableThreads());
#pragma omp parallel for
#endif
            for (int v = 0; v < dispValues.rows; ++v) {
                const float d = dispValues.at<float>(v, u1);
                if (d * 0.0 != 0.0) continue;  // check for NaN

                const cv::Vec3b &c1 = Irl.at<cv::Vec3b>(v, u1);
                if (c1 == blackPx) continue;

                int u2 = u1 - d;
                if (u2 < 0 || u2 >= Irr.cols) continue;

                const cv::Vec3b &c2 = Irr.at<cv::Vec3b>(v, u2);
                if (c2 == blackPx) continue;

                Vec3 unrectified1 = Tlinv * Vec3(u1, v, 1.0);
                Vec3 unrectified2 = Trinv * Vec3(u2, v, 1.0);

                const std::vector<Vec2> x = {unrectified1.topRows(2),
                                             unrectified2.topRows(2)};
                Vec4 Q = utils::triangulate(x, P);

#pragma omp critical
                {
                    p3d::task::progress_ = u1;
                    pts3D.emplace_back(Vec3f(Q(0), Q(1), Q(2)));
                    colors.emplace_back(Vec3uc(c1.val[0], c1.val[1], c1.val[2]));
                }
            }
        }

        Mat3Xf result;
        utils::convert(pts3D, result);

        std::string newPcd = "densePair" + std::to_string(pairIdx);

        if (data.pointCloudCtnr().contains(newPcd)) {
            auto &pcd = data.pointCloudCtnr()[newPcd];
            groupCmd->add(
                new CommandSetProperty{&pcd, p3dPointCloud_vertices, result});
            groupCmd->add(new CommandSetProperty{&pcd, p3dPointCloud_colors, colors});
        } else {
            groupCmd->add(
                new CommandPointCloudAdd(&data.pointCloudCtnr(), newPcd, result, colors));
        }
        LOG_OK("Triangulated (stereo) %i points", result.cols());
    }

    p3d::task::reset();
    if (groupCmd->empty())
        delete groupCmd;
    else
        p3d::cmder::executeCommand(groupCmd);
}

void p3d::triangulateDenseDev(Project &data)
{
    //LOG_ERR("Not implemented yet");
    //return;

    if (data.nbImages() < 2) {
        LOG_ERR("Not enough images");
        return;
    }

    std::vector<Vec3> pts3D;
    for (int pairIdx = 0; pairIdx < data.nbImagePairs(); ++pairIdx) {
        if (!data.imagePair(pairIdx)->hasDisparityMap()) return;
        auto dispValues = data.imagePair(pairIdx)->getDisparityMap().clone();
        if (dispValues.type() != CV_32F) {
            LOG_ERR("Disparity map type must be CV_32F");
            return;
        }
        dispValues = dispValues / 16.0;

        Mat3 Tl = data.imagePair(pairIdx)->getRectifyingTransformL();
        Mat3 Tr = data.imagePair(pairIdx)->getRectifyingTransformR();

        Mat3 Tli = utils::inverseTransform(Tl);
        Mat3 Tri = utils::inverseTransform(Tr);

        LOG_INFO("disp(%i)", dispValues.type());

        // **** build potential matches
        std::vector<std::vector<Vec2>> matches;
        for (int u = 0; u < dispValues.cols; ++u) {
            for (int v = 0; v < dispValues.rows; ++v) {
                const double d = static_cast<double>(dispValues.at<float>(v, u));
                if (d * 0.0 != 0.0) continue;  // check for NaN

                Vec3 pl = Tli * Vec3(u, v, 1.0);
                Vec3 pr = Tri * Vec3(double(u) - d, v, 1.0);

                if (pl(0) < 0) continue;
                if (pl(1) < 0) continue;
                if (pr(0) < 0) continue;
                if (pr(1) < 0) continue;

                //if (pl(0) > im1.width()) continue;
                //if (pl(1) > im1.height()) continue;
                //if (pr(0) > im2.width()) continue;
                //if (pr(1) > im2.height()) continue;

                matches.push_back({pl.topRows(2), pr.topRows(2)});
            }
        }

        auto Ps = data.getCameraMatrices();
        std::vector<Mat34> P;
        P.push_back(Ps[pairIdx]);
        P.push_back(Ps[pairIdx + 1]);

        int nbGoodPoints = 0;
#ifdef WITH_OPENMP
        omp_set_num_threads(utils::nbAvailableThreads());
#pragma omp parallel for
#endif
        for (auto p = 0; p < matches.size(); ++p) {
            Vec4 pt = utils::triangulate(matches[p], P);
            pt /= pt(3);

            // check consistensy
            double err = 0.0;
            for (auto c = 0; c < P.size(); ++c) {
                Vec3 q1 = P[c] * pt;
                q1 /= q1(2);
                const auto &xr = q1(0);
                const auto &yr = q1(1);
                const auto &x = matches[p][c](0);
                const auto &y = matches[p][c](1);
                err += (xr - x) * (xr - x) + (yr - y) * (yr - y);
            }
            bool inlier = err < 4.0;
            if (!inlier) continue;

#pragma omp critical
            {
                pts3D.push_back(pt.topRows(3));
                nbGoodPoints++;
            }
        }
        LOG_INFO("Good points: %0.2f %%", 100.0f * nbGoodPoints / float(matches.size()));
    }

    Mat3Xf result;
    utils::convert(pts3D, result);

    std::string newPcd = "denseMVS";

    if (data.pointCloudCtnr().contains(newPcd)) {
        auto &pcd = data.pointCloudCtnr()[newPcd];
        p3d::cmder::executeCommand(
            new CommandSetProperty{&pcd, p3dPointCloud_vertices, result});
    } else {
        p3d::cmder::executeCommand(
            new CommandPointCloudAdd(&data.pointCloudCtnr(), newPcd, result));
    }
    LOG_OK("Triangulated %i points", pts3D.size());
}

void p3d::bundleAdjustment(Project &data)
{
    if (!data.pointCloudCtnr().contains("sparse")) {
        LOG_ERR("Bundle adjust. needs a sparse cloud");
        return;
    }
    auto &pcd = data.pointCloudCtnr().at("sparse");
    if (pcd.nbPoints() == 0) {
        LOG_ERR("Sparse point cloud is empty");
        return;
    }

    BundleData p;

    std::vector<Landmark> landmarks;
    LandmarksUtil::from3DPtsMeasMat(pcd.getVertices().cast<double>(),
                                    data.getMeasurementMatrix(), landmarks);
    p.landmarks = &landmarks;

    data.getCamerasIntrinsics(&p.cam);
    data.getCamerasExtrinsics(&p.R, &p.t);

    LOG_OK("Bundle adjustement: started...");
    const auto nbCams = p.R.size();
    {
        BundleParams params(nbCams);
        params.setConstAll();
        params.setVaryingPts();
        BundleAdjustment ba;
        ba.run(p, params);
    }

    {
        BundleParams params(nbCams);
        params.setConstAll();
        params.setVaryingAllCams(p3dBundleParam_R);
        params.setVaryingAllCams(p3dBundleParam_t);
        params.setConstAllParams({0});
        BundleAdjustment ba;
        ba.run(p, params);
    }

    {
        BundleParams params(nbCams);
        params.setConstAll();
        params.setVaryingPts();
        BundleAdjustment ba;
        ba.run(p, params);
    }

    Mat3X X;
    LandmarksUtil::toMat3X(landmarks, X);
    landmarks.clear();

    pcd.setVertices(X.cast<float>());
    data.setCamerasIntrinsics(p.cam);
    data.setCamerasExtrinsics(p.R, p.t);

    LOG_OK("Bundle adjustement: done");
    LOG_WARN("Bundle adjustement: no undo");
}

void p3d::exportPLY(const Project &data, const std::string &label,
                    const std::string &filepath)
{
    if (filepath == "") {
        LOG_ERR("Can't export to an empty file");
        return;
    }

    const auto &ctnr = data.getPointCloudCtnr();
    if (!ctnr.contains(label)) {
        LOG_ERR("No %s point cloud...", label.c_str());
        return;
    }
    auto pcd = ctnr.at(label);
    if (pcd.getVertices().cols() == 0) {
        LOG_ERR("Point cloud %s has no points...", label.c_str());
        return;
    }

    LOG_DBG("Exporting pcd: %s", label.c_str());
    try {
        utils::exportToPly(pcd.getVertices(), filepath);
        LOG_OK("Exported point cloud (%s): %i points", label.c_str(),
               pcd.getVertices().cols());
    } catch (...) {
        LOG_ERR("Point cloud export failed");
    }
}

void p3d::exportOpenMVS(const Project &data, const std::string &filepath)
{
    MVS::Interface scene;

    std::vector<Mat3> Rarr;
    data.getCamerasRotations(&Rarr);

    const int nViews = data.nbImages();
    // fill scene.cameras
    for (int i = 0; i < nViews; i++) {
        const auto & im = data.getImage(i);

        MVS::Interface::Platform platform;
        MVS::Interface::Platform::Camera camera;

        camera.name = "camera" + std::to_string(i);
        camera.bandName = "SEM";
        camera.width = im.width();
        camera.height = im.height();

        cv::Matx<double,3,3> Kcv = cv::Matx<double,3,3>::eye();
        const auto K = im.getCamera().getK();
        cv::eigen2cv(K,Kcv);
        camera.K = Kcv;
        camera.R = cv::Matx<double,3,3>::eye();
        camera.C = cv::Point3_<double>(0,0,0);

        platform.cameras.push_back(camera);
        scene.platforms.push_back(platform);
    }

    // ***** fill scene.images

    scene.images.reserve(nViews);
    for (int i = 0; i < nViews; i++) {
        const auto & im = data.getImage(i);

        MVS::Interface::Image image;
        image.name = im.getPath();
        image.platformID = i;
        MVS::Interface::Platform& platform = scene.platforms[image.platformID];
        image.cameraID = 0;

        MVS::Interface::Platform::Pose pose;
        image.poseID = 0;

        cv::Matx<double,3,3> Rcv = cv::Matx<double,3,3>::eye();
        const auto R = Rarr[i];
        cv::eigen2cv(R,Rcv);
        pose.R = Rcv;

        const Vec2 t = im.getTranslation();
        cv::Point3_<double> tcv(t[0],t[1],-1000);
        pose.C = tcv;

        platform.poses.push_back(pose);
        scene.images.emplace_back(image);
    }

    if (!data.getPointCloudCtnr().contains("sparse")) {
        LOG_ERR("no sparse point cloud, aborting...");
        return;
    }

    const auto & pcd = data.getPointCloudCtnr().at("sparse");
    // ***** fill scene.images

    scene.vertices.reserve(pcd.nbPoints());
    for (int pointIdx = 0; pointIdx < pcd.nbPoints(); pointIdx++)
    {
        MVS::Interface::Vertex vert;
        MVS::Interface::Vertex::ViewArr& views = vert.views;

        for (int i = 0; i < nViews; ++i) {
            MVS::Interface::Vertex::View view;
            view.imageID = i;
            view.confidence = 0.0;
            views.push_back(view);
        }

        Vec3f pt = pcd.getVertex(pointIdx);
        vert.X = cv::Point3_<float>(pt[0],pt[1],pt[2]);
        scene.vertices.push_back(vert);
    }

    // write OpenMVS data
    if (!MVS::ARCHIVE::SerializeSave(scene, filepath)) {
        LOG_ERR("Failed to export as openMVS scene");
        return;
    }

    LOG_OK("project saved to openMVS interface format");
}

void p3d::deletePointCloud(Project &data, const char *lbl)
{
    auto ctnr = &data.pointCloudCtnr();
    p3d::cmder::executeCommand(new CommandPointCloudDelete(ctnr, lbl));
}

entt::meta_any p3d::getSetting(Project &project, const p3dSetting &name)
{
    auto s = project.settings();
    auto data = s->resolve().data(P3D_ID_TYPE(name));
    if (!data) return entt::meta_any{nullptr};
    return data.get(*s);
}

void p3d::setSetting(Project &project, const p3dSetting &id, const entt::meta_any &value)
{
    auto s = project.settings();
    p3d::cmder::executeCommand(new CommandSetProperty{s, P3D_ID_TYPE(id), value});
}

void p3d::setProjectProperty(Project &data, const uint32_t &propId,
                             const entt::meta_any &value)
{
    p3d::cmder::executeCommand(new CommandSetProperty{&data, propId, value});
}

void p3d::setImageProperty(Project &data, const P3D_ID_TYPE &propId,
                           const entt::meta_any &value, std::vector<int> imIds)
{
    if (data.nbImages() == 0) return;
    if (imIds.empty())
        for (int i = 0; i < data.nbImages(); ++i) imIds.push_back(i);

    CommandGroup *group = new CommandGroup();
    for (int idx = 0; idx < imIds.size(); idx++)
    {
        auto i  = imIds[idx];
        auto im = data.image(i);
        if (!im) continue;

        group->add(new CommandSetProperty{im, propId, value});
    }

    if (group->empty())
        delete group;
    else
        p3d::cmder::executeCommand(group);
}

void p3d::setImagePairProperty(Project &data, const P3D_ID_TYPE &propId,
                               const entt::meta_any &value, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    CommandGroup *group = new CommandGroup();

    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];
        auto imPair = data.imagePair(i);
        if (!imPair) continue;

        group->add(new CommandSetProperty{imPair, propId, value});
    }

    if (group->empty())
        delete group;
    else
        p3d::cmder::executeCommand(group);
}

void p3d::copyImageProperty(Project &projectData, const P3D_ID_TYPE &propId, int from,
                            const std::vector<int> &to)
{
    auto imFrom = projectData.image(from);
    if (imFrom == nullptr) return;

    auto data = entt::resolve<Image>().data(propId);
    if (!data) {
        LOG_ERR("Can't copy setting that doesn't exist");
        return;
    }

    auto setting = data.get(*imFrom);

    auto toVec = to;
    if (toVec.empty()) {
        for (auto i = 0; i < projectData.nbImages(); ++i) {
            if (i != from) toVec.push_back(i);
        }
    }

    CommandGroup *groupCmd = new CommandGroup();

    for (const auto &idx : toVec) {
        auto imPair = projectData.image(idx);
        if (imPair == nullptr) continue;

        groupCmd->add(new CommandSetProperty{imPair, propId, setting});
    }

    if (groupCmd->empty())
        delete groupCmd;
    else
        p3d::cmder::executeCommand(groupCmd);

    std::string output = "Copied settings from image (" + std::to_string(from) + ") to " +
                         utils::to_string(toVec);

    LOG_DBG("%s", output.c_str());
}

void p3d::copyImagePairProperty(Project &projectData, const P3D_ID_TYPE &propId, int from,
                                const std::vector<int> &to)
{
    auto imFrom = projectData.imagePair(from);
    if (imFrom == nullptr) return;

    auto data = entt::resolve<ImagePair>().data(propId);
    if (!data) {
        LOG_ERR("Can't copy setting that doesn't exist");
        return;
    }

    auto setting = data.get(*imFrom);

    auto toVec = to;
    if (toVec.empty()) {
        for (int i = 0; i < projectData.nbImagePairs(); ++i) {
            if (i != from) toVec.push_back(i);
        }
    }

    CommandGroup *groupCmd = new CommandGroup();

    for (const auto &idx : toVec) {
        auto imPair = projectData.imagePair(idx);
        if (imPair == nullptr) continue;

        groupCmd->add(new CommandSetProperty{imPair, propId, setting});
    }

    if (groupCmd->empty())
        delete groupCmd;
    else
        p3d::cmder::executeCommand(groupCmd);

    std::string output = "Copied settings from image pair (" +
                         std::to_string(from) + ") to " +
                         utils::to_string(toVec);

    LOG_DBG("%s", output.c_str());
}

void p3d::undo()
{
    if (p3d::cmder::get())
        p3d::cmder::get()->undoCommand();
}
void p3d::redo()
{
    if (p3d::cmder::get())
        LOG_WARN("Not implemented");
}

void p3d::mergeNextCommand()
{
    if (p3d::cmder::get())
        p3d::cmder::get()->mergeNextCommand();
}
