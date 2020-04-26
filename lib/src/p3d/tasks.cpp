#include "tasks.h"

#include <omp.h>

#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <map>
#include <vector>

#include "p3d/data/image.h"
#include "p3d/logger.h"
#include "p3d/multiview/autocalib.h"
#include "p3d/multiview/bundle_adjustment.h"
#include "p3d/stereo/fundmat.h"
#include "p3d/stereo/rectification.h"
#include "p3d/utils.h"

#include "p3d/command_manager.h"

#define P3D_PROJECT_EXTENSION ".yml.gz"

using namespace p3d;

namespace p3d::task {
int total_{0};
int progress_{0};
std::string name_{""};

void reset()
{
    total_ = 0;
    progress_ = 0;
    name_ = "";
}

int total()
{
    return total_;
}

int progress()
{
    return progress_;
}

std::string name()
{
    return name_;
}

float progressPercent()
{
    if (total_ == 0) return 1.0f;
    return float(progress_) / float(total_);
}

} // namespace p3d::task

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

    p3d::task::name_ = "Feature extraction";
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
            LOG_ERR("Features cannot be extracted: no image loaded");
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

bool p3d::matchFeatures(Project &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return false;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    int nbPairs = imPairsIds.size();

    p3d::task::name_ = "Matching";
    p3d::task::total_ = nbPairs;
    p3d::task::progress_ = 0;

    CommandGroup *groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(
        std::min(int(data.nbImagePairs()), utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (int i = 0; i < nbPairs; i++) {
        auto imPair = data.imagePair(imPairsIds[i]);
        if (!imPair || !imPair->isValid()) continue;

        auto imIdxL = imPair->imL();
        auto imIdxR = imPair->imR();
        auto imL = data.image(imIdxL);
        auto imR = data.image(imIdxR);

        if (imR == nullptr || imR == nullptr) continue;
        if (!imL->hasFeatures() || !imR->hasFeatures()) {
            LOG_ERR("Features must be extracted before matching");
            continue;
        }

        MatchingPars pars = imPair->getMatchingPars();

        const auto &_descriptorsLeftImage = imL->getDescriptors();
        const auto &_descriptorsRightImage = imR->getDescriptors();
        std::vector<Match> matchesPair;
        MatchingUtil::match(_descriptorsLeftImage, _descriptorsRightImage, pars,
                            matchesPair);

        if (matchesPair.empty()) {
            LOG_ERR("Pair %i, matching failed", imPairsIds[i]);
            continue;
        }

        auto cmd = new CommandSetProperty{imPair, p3dImagePair_matches, matchesPair};

#pragma omp critical
        {
            p3d::task::progress_++;
            groupCmd->add(cmd);
            LOG_OK("Pair %i, matched %i features", imPairsIds[i], matchesPair.size());
            if (!cmd->isValid())
                LOG_WARN("Pair %i, matches didn't change", imPairsIds[i]);
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

bool p3d::findFundamentalMatrix(Project &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return false;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    p3d::task::name_ = "Epipolar geometry";
    p3d::task::total_ = imPairsIds.size();
    p3d::task::progress_ = 0;

    CommandGroup *groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(imPairsIds.size()), utils::nbAvailableThreads()));
#pragma omp parallel for
#endif
    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];
        std::vector<Vec2> ptsL, ptsR;
        data.getPairwiseMatches(i, ptsL, ptsR);
        if (ptsL.empty() || ptsR.empty()) {
            LOG_OK("Pair %i has no matches", i);
            continue;
        }

        Mat3 F = FundMatUtil::findAffineCeres(ptsL, ptsR);
        auto angles = FundMatUtil::slopAngles(F);
        double theta1 = angles.first;
        double theta2 = angles.second;

        auto cmd1 = new CommandSetProperty{data.imagePair(i), p3dImagePair_fundMat, F};
        auto cmd2 =
            new CommandSetProperty{data.imagePair(i), p3dImagePair_Theta1, theta1};
        auto cmd3 =
            new CommandSetProperty{data.imagePair(i), p3dImagePair_Theta2, theta2};
#pragma omp critical
        {
            p3d::task::progress_++;

            groupCmd->add(cmd1);
            groupCmd->add(cmd2);
            groupCmd->add(cmd3);

            if (cmd1->isValid())
                LOG_OK("Pair %i, estimated F (ceres)", i);
            else
                LOG_WARN("Pair %i, F didn't change", imPairsIds[i]);
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

        if (utils::floatEq(angleL, 0.0) && utils::floatEq(angleR, 0.0))
            continue;

        std::vector<Vec2> ptsL, ptsR;
        data.getPairwiseMatches(i, ptsL, ptsR);

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

        cv::Mat disparityMap;
        DenseMatchingUtil::findDisparity(
            imLeftR, imRightR, disparityMap,
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

void p3d::findMeasurementMatrixFull(Project &data)
{
    Mat Wfull;
    auto nbIm = data.nbImages();
    auto nbPairs = data.nbImagePairs();
    if (nbPairs == 0) return;

    Mati table;
    std::vector<std::map<int, int>> matchesMaps;
    matchesMaps.resize(nbPairs);
    for (auto i = 0; i < nbPairs; i++) {
        auto imPair = data.imagePair(i);
        if (!imPair) continue;
        imPair->getMatchesAsMap(matchesMaps[i]);
    }

    utils::matchesMapsToTable(matchesMaps, table);

    auto nbLandmarks = table.cols();
    Wfull.setZero(3 * nbIm, nbLandmarks);
    for (int i = 0; i < nbIm; ++i) {
        auto image = data.image(i);
        if (!image) continue;
        auto kpts = image->getKeyPoints();
        for (int p = 0; p < nbLandmarks; ++p) {
            auto ptIdx = table(i, p);
            if (ptIdx < 0) continue;
            if (ptIdx >= kpts.size()) {
                LOG_ERR("Full measurement matrix: wrong feature indices");
                // most certainly features were reestimated afrer matches
                return;
            }
            const double x = static_cast<double>(kpts[ptIdx].pt.x);
            const double y = static_cast<double>(kpts[ptIdx].pt.y);
            Wfull.block(3 * i, p, 3, 1) << x, y, 1.0;
        }
    }
    //data.setMeasurementMatrixFull(Wfull);
    p3d::cmder::executeCommand(
        new CommandSetProperty{&data, P3D_ID_TYPE(p3dProject_measMatFull), Wfull});
    LOG_OK("Full measurement matrix: %ix%i", Wfull.rows(), Wfull.cols());
}

void p3d::findMeasurementMatrix(Project &data)
{
    const auto &Wf = data.getMeasurementMatrixFull();
    if (Wf.cols() == 0 || Wf.rows() == 0) {
        LOG_ERR("Full measurement matrix must be estimated first");
        return;
    }

    Mat W;
    W.setZero(Wf.rows(), 0);
    for (auto c = 0; c < Wf.cols(); ++c) {
        if (std::abs(Wf.col(c).prod()) > 1e-5) {  // there is no zeros in the column
            W.conservativeResize(Eigen::NoChange, W.cols() + 1);
            W.rightCols(1) = Wf.col(c);
        }
    }

    //data.setMeasurementMatrix(W);
    p3d::cmder::executeCommand(
        new CommandSetProperty{&data, P3D_ID_TYPE(p3dProject_measMat), W});
    LOG_OK("Measurement matrix: %ix%i", W.rows(), W.cols());
}

bool p3d::autocalibrate(Project &data, std::vector<int> camIds)
{
    if (camIds.empty()) {
        for (int i = 0; i < data.nbImages(); i++) camIds.emplace_back(i);
    }

    int nbImgs = camIds.size();
    if (nbImgs < 3) {
        LOG_ERR("autocalibration needs at least 3 images");
        return false;
    }

    auto W = data.getMeasurementMatrixFull();
    if (W.cols() == 0 || W.rows() == 0) {
        LOG_ERR("Meas mat should be estimated before autocalibration");
        return false;
    }

    std::vector<int> selectedCols;
    Mat Wfsub;
    utils::findFullSubMeasurementMatrix(W,Wfsub,camIds,&selectedCols);
    if (Wfsub.cols() == 0 || Wfsub.rows() == 0) {
        LOG_ERR("findFullSubMeasurementMatrix failed");
        return false;
    }

    Mat2X slopes;
    slopes.setZero(2, nbImgs);
    for (auto i = 0; i < nbImgs - 1; ++i) {
        auto imPair = data.imagePair(camIds[i]);
        if (imPair->imL() != camIds[i]) {
            LOG_ERR("Pair doesn't correspond to the index");
            return -1;
        }
        if (imPair->imR() != camIds[i+1]) {
            LOG_ERR("Pair doesn't correspond to the index");
            return -1;
        }

        slopes(0, i + 1) = data.imagePair(camIds[i])->getTheta1();
        slopes(1, i + 1) = data.imagePair(camIds[i])->getTheta2();
    }

    AutoCalibrator autocalib(nbImgs);

    autocalib.setMaxTime(60);

    autocalib.setMeasurementMatrix(Wfsub);
    autocalib.setSlopeAngles(slopes);
    autocalib.run();

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "Calibration     :\n"
              << autocalib.getCalibrationMatrix().format(CleanFmt) << std::endl;
    std::cout << "Camera matrices :\n"
              << utils::concatenateMat(autocalib.getCameraMatrices()).format(CleanFmt) << std::endl;

    AffineCamera c(autocalib.getCalibrationMatrix());

    auto tvec = autocalib.getTranslations();
    auto ti = data.getImage(camIds[0]).getTranslation();
    bool correctForTranslation1stCam = (ti.norm() > 1e-2);
    if (correctForTranslation1stCam) {
        std::vector<Vec2> x;
        auto P = autocalib.getCameraMatrices();
        for (int i = 0; i < P.size(); i++) x.push_back(tvec[i]);
        Vec4 X = utils::triangulate(x,P);
        X /= X.w();
        X(0) -= tvec[0](0) - ti(0);
        X(1) -= tvec[0](1) - ti(1);
        for (int i = 0; i < P.size(); i++) {
            Vec3 q = P[i] * X;
            q = q / q(2);
            tvec[i](0) = q(0);
            tvec[i](1) = q(1);
        }
    }

    for (auto n = 0; n < nbImgs; ++n) {
        const auto i = camIds[n];
        AffineCamera c;
        data.image(i)->setCamera(c);
        data.image(i)->setTranslation(tvec[n]);
    }

    auto rotRad = autocalib.getRotationAngles();
    // first line of rotRad is [0,0,0]
    for (auto n = 0; n < nbImgs - 1; ++n) {
        const auto i = camIds[n];
        data.imagePair(i)->setTheta1(rotRad(n + 1, 0));
        data.imagePair(i)->setRho(rotRad(n + 1, 1));
        data.imagePair(i)->setTheta2(rotRad(n + 1, 2));
    }

    auto rot = utils::rad2deg(rotRad);

    LOG_OK("Angles: [theta rho theta']");
    std::stringstream ss;
    ss << rot.format(CleanFmt);
    auto rows = utils::split(ss.str(), "\n");
    for (auto i = 0; i < rot.rows(); ++i) {
        LOG_OK("Pair %i: %s", i, rows[i].c_str());
    }

    std::cout << "Rotation angles :\n"
              << rot.format(CleanFmt) << std::endl;
}

void p3d::triangulateSparse(Project &data)
{
    LOG_DBG("MAKE BUTTON DISABLED IF NO CALIB + NO FULL W");

    auto Wf = data.getMeasurementMatrixFull();
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
        float sinRho = std::sin(rho);

        auto dispValues = data.imagePair(pairIdx)->getDisparityMap().clone();
        if (dispValues.type() != CV_32F) {
            LOG_ERR("pair %i, disparity map type must be CV_32F", pairIdx);
            continue;
        }
        dispValues = dispValues / 16.0;

        std::vector<Vec3f> pts3D;
        std::vector<Vec3f> colors;

        cv::Mat I = data.imagePair(pairIdx)->getRectifiedImageL();
        Mat3 Tl = data.imagePair(pairIdx)->getRectifyingTransformL().inverse();
        Mat34 P = data.getCameraMatrices()[imPair->imL()];

        if (I.type() != CV_8UC3) LOG_DBG("Rectified image has wrong type: %i", I.type());

        p3d::task::name_ = "Trianulation, pair " + std::to_string(pairIdx);
        p3d::task::total_ = dispValues.cols;
        p3d::task::progress_ = 0;

        for (int u = 0; u < dispValues.cols; ++u) {
#ifdef WITH_OPENMP
        omp_set_num_threads(utils::nbAvailableThreads());
#pragma omp parallel for
#endif
            for (int v = 0; v < dispValues.rows; ++v) {
                const float d = dispValues.at<float>(v, u);
                if (d * 0.0 != 0.0) continue;  // check for NaN

                Vec3 unrectified = Tl * Vec3(u, v, 1.0);
                const float q1x = static_cast<float>(unrectified(0));
                const float q1y = static_cast<float>(unrectified(1));
                const float q2x = q1x + d;

                const float q1z = -(q1x * cosRho - q2x) / sinRho;

                //            #pragma omp critical
                cv::Vec3b c = I.at<cv::Vec3b>(v, u);
                if (c == cv::Vec3b(0, 0, 0)) continue;

                #pragma omp critical
                {
                    p3d::task::progress_ = u;
                    pts3D.emplace_back(Vec3f(q1x - P(0, 3), q1y - P(1, 3), q1z));
                    colors.emplace_back(
                        Vec3f(c.val[0] / 255.f, c.val[1] / 255.f, c.val[2] / 255.f));
                }
            }
        }

        Mat3Xf result;
        Mat3Xf colorsMat;
        utils::convert(pts3D, result);
        utils::convert(colors, colorsMat);

        std::string newPcd = "densePair" + std::to_string(pairIdx);

        if (data.pointCloudCtnr().contains(newPcd)) {
            auto &pcd = data.pointCloudCtnr()[newPcd];
            groupCmd->add(
                new CommandSetProperty{&pcd, p3dPointCloud_vertices, result});
            groupCmd->add(
                new CommandSetProperty{&pcd, p3dPointCloud_colors, colorsMat});
        } else {
            groupCmd->add(new CommandPointCloudAdd(
                &data.pointCloudCtnr(), newPcd, result, colorsMat));
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

        Mat3 Tli = Tl.inverse();
        Mat3 Tri = Tr.inverse();

        LOG_INFO("disp(%i)", dispValues.type());

        // **** build potential matches
        std::vector<std::vector<Vec2>> matches;
        for (int u = 0; u < dispValues.cols; ++u) {
            for (int v = 0; v < dispValues.rows; ++v) {
                const double d = static_cast<double>(dispValues.at<float>(v, u));
                if (d * 0.0 != 0.0) continue;  // check for NaN

                Vec3 pl = Tli * Vec3(u, v, 1.0);
                Vec3 pr = Tri * Vec3(double(u) + d, v, 1.0);

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
        //    #ifdef WITH_OPENMP
        //        omp_set_num_threads(utils::nbAvailableThreads());
        //        #pragma omp parallel for
        //    #endif
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
            bool inlier = err < 2 * 2;
            //#pragma omp critical
            if (inlier) {
                pts3D.push_back(pt.topRows(3));
                nbGoodPoints++;
            }
        }
        LOG_INFO("Good points: %0.2f %%", 100.0f * nbGoodPoints / float(matches.size()));
    }

    Mat3Xf result;
    utils::convert(pts3D, result);

    //    p3d::cmder::executeCommand(
    //        new CommandSetProperty{&data, P3D_ID_TYPE(p3dProject_pts3DDense),
    //        result, true));

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
    p.X.setOnes(4, pcd.nbPoints());
    p.X.topRows(3) = pcd.getVertices().cast<double>();
    p.W = data.getMeasurementMatrixFull();

    if (p.X.cols() != p.W.cols()) {
        LOG_ERR("Measurement matrix doesn't correspond to 3D points");
        return;
    }

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

    pcd.setVertices(p.X.topRows(3).cast<float>());
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
