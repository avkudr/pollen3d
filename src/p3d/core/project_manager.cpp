#include "project_manager.h"

#include <omp.h>

#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <vector>
#include <map>

#include "p3d/core/data/image.h"
#include "p3d/core/logger.h"
#include "p3d/core/multiview/autocalib.h"
#include "p3d/core/multiview/bundle_adjustment.h"
#include "p3d/core/utils.h"

#define P3D_PROJECT_EXTENSION ".yml.gz"

ProjectManager *ProjectManager::m_instance = nullptr;

void ProjectManager::loadImages(ProjectData *list, const std::vector<std::string> &imPaths)
{
    if (!list) return;
    if (imPaths.empty()) {
        LOG_ERR("Image list is empty");
        return;
    }

    const size_t nbIm = imPaths.size();
    list->clear();

    std::vector<Image> imgs;
    LOG_INFO("Loading %i images...", nbIm);
    for (size_t i = 0; i < nbIm; ++i){
        Image m(imPaths[i]);
        if (m.isValid()) imgs.emplace_back(m);
    }

    if (imgs.empty()) {
        LOG_ERR("Image loading failed");
        return;
    }

    LOG_OK("Loaded %i/%i", imgs.size(), imPaths.size());
    LOG_WARN("No Undo functionnality");

    list->setImageList(imgs);
}

void ProjectManager::saveProject(ProjectData *data, std::string path) {

    if (!data) return;
    if (path == "") {
        LOG_ERR("Can't save the project to empty path");
        return;
    }

    data->setProjectPath(path);
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "ProjectSettings" << "{";
        m_settings.write(fs);
        fs << "}";
        fs << "ProjectData" << "{";
        data->write(fs);
        fs << "}";
        LOG_OK( "Project %s succesfully saved", path.c_str() );
        fs.release();
    } else {
        LOG_ERR( "Can't write to %s", path.c_str() );
    }
}

void ProjectManager::closeProject(ProjectData *data)
{
    if (data)
        *data = ProjectData();
}

void ProjectManager::openProject(ProjectData *data, std::string path)
{
    if (!data) return;
    if (path == "") return;
    if (!utils::endsWith(path,P3D_PROJECT_EXTENSION)) return;

    closeProject(data);

    LOG_OK( "Loading %s", path.c_str() );

    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            m_settings.read(fs["ProjectSettings"]);
            data->read(fs["ProjectData"]);
            LOG_OK( "Done" );
            fs.
            fs.release();
        } else {
            LOG_ERR( "Can't write to %s", path.c_str() );
        }
    } catch (const cv::Exception & e) {
        LOG_ERR("cvErr:%s",e.what());
    } catch (...) {
        LOG_ERR("Unknown error");
    }
}

void ProjectManager::extractFeatures(ProjectData &data, std::vector<int> imIds)
{
    if (data.nbImages() == 0) return;
    if (imIds.empty())
        for (int i = 0; i < data.nbImages(); ++i) imIds.push_back(i);

    auto nbImgs = static_cast<int>(imIds.size());
#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(nbImgs,utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (auto i = 0; i < nbImgs; i++) {
        Image * im = data.image(imIds[i]);
        if (!im) continue;
        if (im->cvMat().empty()){
            LOG_ERR("Features cannot be extracted: no image loaded");
            continue;
        }

        std::vector<cv::KeyPoint> kpts;
        cv::Mat desc;

        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(
                    getSetting(p3dSetting_featuresDescType).cast<int>(),
                    getSetting(p3dSetting_featuresDescSize).cast<int>(),
                    getSetting(p3dSetting_featuresDescChannels).cast<int>(),
                    getSetting(p3dSetting_featuresThreshold).cast<float>()
                    );
        akaze->detectAndCompute(im->cvMat(), cv::noArray(), kpts, desc);
        akaze.release();

        im->setKeyPoints(kpts);
        im->setDescriptors(desc);

        LOG_OK("%s: extracted %i features", im->name().c_str(), int(kpts.size()));

        kpts.clear();
        desc.release();
    }
}

void ProjectManager::matchFeatures(ProjectData &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    int nbPairs = imPairsIds.size();

    float filterCoef = getSetting(p3dSetting_matcherFilterCoef).cast<float>();

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(data.nbImagePairs()),utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (int i = 0; i < nbPairs; i++) {
        auto imPair = data.imagePair(imPairsIds[i]);
        if (!imPair || !imPair->isValid()) continue;

        auto imIdxL = imPair->imL();
        auto imIdxR = imPair->imR();
        auto imL = data.image(imIdxL);
        auto imR = data.image(imIdxR);

        if ( imR == nullptr || imR == nullptr ) continue;
        if ( !imL->hasFeatures() || !imR->hasFeatures()) {
            LOG_ERR("Features must be extracted before matching");
            continue;
        }
        std::vector<std::vector<cv::DMatch>> poor_matches;
        std::vector<cv::DMatch> matches;
        std::vector<Match> matchesPair;
        const auto & _descriptorsLeftImage  = imL->getDescriptors();
        const auto & _descriptorsRightImage = imR->getDescriptors();

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-L1");
        matcher->knnMatch(_descriptorsLeftImage, _descriptorsRightImage, poor_matches, 2); // 2  best matches

        for(int im = 0; im < cv::min(_descriptorsLeftImage.rows-1,(int) poor_matches.size()); im++){
            if((poor_matches[im][0].distance < filterCoef*(poor_matches[im][1].distance)) &&
                    ((int) poor_matches[im].size()<=2 && (int) poor_matches[im].size()>0)){
                matches.push_back(poor_matches[im][0]);

                matchesPair.emplace_back(
                            Match(
                                poor_matches[im][0].queryIdx,
                                poor_matches[im][0].trainIdx,
                                poor_matches[im][0].distance
                                ));
            }
        }

        imPair->setMatches(matchesPair);

        #pragma omp critical
        {
            LOG_OK("Pair %i, matched %i features", imPairsIds[i], matchesPair.size());
        }
    }

    LOG_WARN("No Undo functionnality");
}

void ProjectManager::findFundamentalMatrix(ProjectData &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    CommandGroup * groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(imPairsIds.size()),utils::nbAvailableThreads()));
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

        Mat3 F = fundmat::findAffineCeres(ptsL,ptsR);
        auto angles = fundmat::slopAngles(F);
        double theta1 = angles.first;
        double theta2 = angles.second;

        auto cmd1 = new CommandSetProperty(data.imagePair(i),p3dImagePair_fundMat,F);
        auto cmd2 = new CommandSetProperty(data.imagePair(i),p3dImagePair_Theta1,theta1);
        auto cmd3 = new CommandSetProperty(data.imagePair(i),p3dImagePair_Theta2,theta2);
        #pragma omp critical
        {
            groupCmd->add(cmd1);
            groupCmd->add(cmd2);
            groupCmd->add(cmd3);
            LOG_OK("Pair %i, estimated F (ceres)", i);
        }
    }

    if (groupCmd->empty()) delete groupCmd;
    else CommandManager::get()->executeCommand(groupCmd);
}

void ProjectManager::rectifyImagePairs(ProjectData &data, std::vector<int> imPairsIds)
{
    LOG_INFO("ProjectManager::rectifyImagePairs");

    if (data.nbImagePairs() == 0) {
        LOG_ERR("No image pair to rectify");
        return;
    }

    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) {
            LOG_INFO("rectifyImagePairs: %i", i);
            imPairsIds.push_back(i);
        }

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(imPairsIds.size()),utils::nbAvailableThreads()));
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

        if (utils::floatEq(angleL,0.0) && utils::floatEq(angleR,0.0)) continue;

        Mat3 Tl,Tr;
        cv::Mat imLrect, imRrect;
        {
            Tr.setIdentity(3,3);
            Tl.setIdentity(3,3);

            cv::Mat Rl = cv::getRotationMatrix2D(cv::Point2d(imL->width()/2.0,imL->height()/2.0),angleL*180.0/CV_PI,1);
            cv::Mat Rr = cv::getRotationMatrix2D(cv::Point2d(imR->width()/2.0,imR->height()/2.0),angleR*180.0/CV_PI,1);

            cv::warpAffine(imL->cvMat(), imLrect, Rl, imL->size());
            cv::warpAffine(imR->cvMat(), imRrect, Rr, imR->size());
            for (int u = 0; u < 2; u++) {
                for (int v = 0; v < 3; v++) {
                    Tl(u,v) = Rl.at<double>(u,v);
                    Tr(u,v) = Rr.at<double>(u,v);
                }
            }
        }

        std::vector<Vec2> ptsL, ptsR;
        data.getPairwiseMatches(i,ptsL,ptsR);

        std::vector<Vec2> ptsLrect, ptsRrect;
        ptsLrect.reserve(ptsL.size());
        ptsRrect.reserve(ptsR.size());

        Vec rectErrors;
        rectErrors.setZero(ptsL.size(),1);

        for(int i = 0; i < ptsL.size(); i++) {
            Vec3 tempPoint;
            tempPoint << ptsL[i][0], ptsL[i][1], 1;
            tempPoint = Tl * tempPoint;
            ptsLrect.emplace_back(Vec2(tempPoint(0),tempPoint(1)));

            tempPoint << ptsR[i][0], ptsR[i][1], 1;
            tempPoint = Tr * tempPoint;
            ptsRrect.emplace_back(Vec2(tempPoint(0),tempPoint(1)));

            rectErrors(i) = ptsLrect[i][1] - ptsRrect[i][1];
        }

        double meanErr = rectErrors.mean();

        cv::Mat imLrectShifted;
        cv::Mat imRrectShifted;
        int shift = std::round(std::abs(meanErr));

        Mat3 Tshift;
        Tshift.setIdentity(3,3);
        Tshift(1,2) = shift;

        if (meanErr > 0){ // features of 2nd are higher than the 1st - shift to the second
            imLrectShifted = imLrect;
            imRrectShifted = cv::Mat::zeros(shift,imRrect.cols, imRrect.type());
            imLrectShifted.push_back(imRrectShifted);
            imRrectShifted.push_back(imRrect);
            for (int i = 0; i < ptsRrect.size(); i++) {
                ptsRrect[i][1] += shift;
            }
            Tr = Tr * Tshift;
        }
        else{
            imRrectShifted = imRrect;
            imLrectShifted = cv::Mat::zeros(shift,imLrect.cols, imLrect.type());
            imRrectShifted.push_back(imLrectShifted);
            imLrectShifted.push_back(imLrect);
            for (int i = 0; i < ptsLrect.size(); i++) {
                ptsLrect[i][1] += shift;
            }
            Tl = Tl * Tshift;
        }

        for(int i = 0; i < ptsL.size(); i++)
            rectErrors(i) = ptsLrect[i][1] - ptsRrect[i][1];

        imLrect = imLrectShifted;
        imRrect = imRrectShifted;

        imPair->setRectifyingTransformL(Tl);
        imPair->setRectifyingTransformR(Tr);
        imPair->setRectifiedImageL(imLrect);
        imPair->setRectifiedImageR(imRrect);

        #pragma omp critical
        {
            LOG_OK("Pair %i, rectification... done", i);
            LOG_INFO("- error mean: %.3f", rectErrors.mean());
            LOG_INFO("- error std: %.3f", std::sqrt((rectErrors.array() - rectErrors.mean()).square().sum()/(rectErrors.size()-1)));
            LOG_INFO("- angleL: %.3f deg", angleL*180.0/CV_PI);
            LOG_INFO("- angleR: %.3f deg", angleR*180.0/CV_PI);
        }
    }

    LOG_WARN("No Undo functionnality");
}

void ProjectManager::findDisparityMap(ProjectData &data, std::vector<int> imPairsIds)
{
    int _method = cv::StereoSGBM::MODE_SGBM; // Default method

    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

    CommandGroup * groupCmd = new CommandGroup();

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(imPairsIds.size()),utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (int idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];
        if (!data.imagePair(i)) continue;
        if (!data.imagePair(i)->isRectified()) continue;

        const auto & imLeftR = data.imagePair(i)->getRectifiedImageL();
        const auto & imRightR = data.imagePair(i)->getRectifiedImageR();

        const auto & lowerBound = 16*data.imagePair(i)->denseMatching.dispLowerBound;
        const auto & upperBound = 16*data.imagePair(i)->denseMatching.dispUpperBound;
        const auto & blockSize  = data.imagePair(i)->denseMatching.dispBlockSize;

        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create( lowerBound,
                                                               upperBound, //number of disparities
                                                               blockSize);
        sgbm->setMode(_method);

        int cn = imLeftR.channels();
        sgbm->setP1(8*cn*blockSize*blockSize);
        sgbm->setP2(32*cn*blockSize*blockSize);

        try{
            cv::Mat disparityMap;
            sgbm->compute(imLeftR, imRightR, disparityMap);

            if (disparityMap.type() != CV_32F)
                disparityMap.convertTo(disparityMap,CV_32F);

            //cv::bilateralFilter(temp,_dispValues,21,180,180); //use bilateral filter ?
            //_dispValues = temp; // no filter

            auto cmd = new CommandSetPropertyCV(
                        data.imagePair(i),
                        &ImagePair::setDisparityMap,&ImagePair::getDisparityMap,disparityMap);

            #pragma omp critical
            {
                groupCmd->add(cmd);
                LOG_OK("Pair %i, estimated disparity", i);
            }
        } catch(...) {
            LOG_ERR("Pair %i, estimated disparity failed", i);
        }
    }

    if (groupCmd->empty()) delete groupCmd;
    else CommandManager::get()->executeCommand(groupCmd);
}

void ProjectManager::filterDisparityBilateral(ProjectData &data, std::vector<int> imPairsIds)
{
    auto pairIdx = 0;
    auto imPair = data.imagePair(pairIdx);
    if (!imPair) return;
    if (!imPair->hasDisparityMap()) return;

    auto m = imPair->getDisparityMap().clone();
    auto d = imPair->denseMatching.bilateralD;
    auto sc = imPair->denseMatching.bilateralSigmaColor;
    auto ss = imPair->denseMatching.bilateralSigmaSpace;

    cv::Mat mFiltered;

    LOG_DBG("Disparity map type: %i", m.type());

    if (m.type() != CV_32F)
        m.convertTo(m,CV_32F);

    LOG_DBG("Disparity map type: %i", m.type());

    cv::bilateralFilter(m,mFiltered,d,sc,ss); //use bilateral filter ?

    auto cmd = new CommandSetPropertyCV(
                imPair,
                &ImagePair::setDisparityMap,&ImagePair::getDisparityMap,mFiltered);
    CommandManager::get()->executeCommand(cmd);
    LOG_OK("Filtered disparity (bilateral)");
}

void ProjectManager::filterDisparitySpeckles(ProjectData &data, std::vector<int> imPairsIds)
{

}

void ProjectManager::findMeasurementMatrixFull(ProjectData &data)
{
    Mat Wfull;
    auto nbIm = data.nbImages();
    auto nbPairs = data.nbImagePairs();
    if (nbPairs == 0) return;

    Mati table;
    std::vector<std::map<int,int>> matchesMaps;
    matchesMaps.resize(nbPairs);
    for (auto i = 0 ; i < nbPairs; i++){
        auto imPair = data.imagePair(i);
        if (!imPair) continue;
        imPair->getMatchesAsMap(matchesMaps[i]);
    }

    utils::matchesMapsToTable(matchesMaps,table);

    auto nbLandmarks = table.cols();
    Wfull.setZero(3*nbIm,nbLandmarks);
    for (int i = 0; i < nbIm; ++i){
        auto image = data.image(i);
        if (!image) continue;
        auto kpts = image->getKeyPoints();
        for (int p = 0; p < nbLandmarks; ++p){
            auto ptIdx = table(i,p);
            if (ptIdx < 0) continue;
            if (ptIdx >= kpts.size()) {
                LOG_ERR("Full measurement matrix: wrong feature indices");
                // most certainly features were reestimated afrer matches
                return;
            }
            const double x = static_cast<double>(kpts[ptIdx].pt.x);
            const double y = static_cast<double>(kpts[ptIdx].pt.y);
            Wfull.block(3*i,p,3,1) << x,y,1.0;
        }
    }
    //data.setMeasurementMatrixFull(Wfull);
    CommandManager::get()->executeCommand(
                new CommandSetProperty(&data,P3D_ID_TYPE(p3dData_measMatFull),Wfull)
                );
    LOG_OK("Full measurement matrix: %ix%i",Wfull.rows(),Wfull.cols());
}

void ProjectManager::findMeasurementMatrix(ProjectData &data)
{
    const auto & Wf = data.getMeasurementMatrixFull();
    if (Wf.cols() == 0 || Wf.rows() == 0) {
        LOG_ERR("Full measurement matrix must be estimated first");
        return;
    }

    Mat W;
    W.setZero(Wf.rows(),0);
    for (auto c = 0; c < Wf.cols(); ++c){
        if (std::abs(Wf.col(c).prod()) > 1e-5){ // there is no zeros in the column
            W.conservativeResize(Eigen::NoChange, W.cols()+1);
            W.rightCols(1) = Wf.col(c);
        }
    }

    //data.setMeasurementMatrix(W);
    CommandManager::get()->executeCommand(
                new CommandSetProperty(&data,P3D_ID_TYPE(p3dData_measMat),W)
                );
    LOG_OK("Measurement matrix: %ix%i",W.rows(),W.cols());
}

void ProjectManager::autocalibrate(ProjectData &data)
{
    auto W = data.getMeasurementMatrix();
    if (W.cols() == 0 || W.rows() == 0) {
        LOG_ERR("Meas mat should be estimated before autocalibration");
        return;
    }

    Mat2X slopes;
    slopes.setZero(2,data.nbImages());
    for (auto i = 0; i < data.nbImagePairs(); ++i) {
        slopes(0,i+1) = data.imagePair(i)->getTheta1();
        slopes(1,i+1) = data.imagePair(i)->getTheta2();
    }

    AutoCalibrator autocalib(data.nbImages());

    autocalib.setMaxTime( 60 );

    autocalib.setMeasurementMatrix(W);
    autocalib.setSlopeAngles(slopes);
    autocalib.run();

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "Calibration     :\n" << autocalib.getCalibrationMatrix().format(CleanFmt) << std::endl;
    std::cout << "Camera matrices :\n" << utils::concatenateMat(autocalib.getCameraMatrices()).format(CleanFmt) << std::endl;

    AffineCamera c(autocalib.getCalibrationMatrix());
    auto tvec = autocalib.getTranslations();

    for (auto i = 0; i < data.nbImages(); ++i) {
        AffineCamera c;
        data.image(i)->setCamera(c);
        data.image(i)->setTranslation(tvec[i]);
    }

    auto rotRad = autocalib.getRotationAngles();
    // first line of rotRad is [0,0,0]
    for (auto i = 0; i < data.nbImagePairs(); ++i) {
        data.imagePair(i)->setTheta1(rotRad(i+1,0));
        data.imagePair(i)->setRho(rotRad(i+1,1));
        data.imagePair(i)->setTheta2(rotRad(i+1,2));
    }

    auto rot = utils::rad2deg(rotRad);

    LOG_OK("Angles: [theta rho theta']");
    std::stringstream ss;
    ss << rot.format(CleanFmt);
    auto rows = utils::split(ss.str(),"\n");
    for (auto i = 0; i < rot.rows(); ++i) {
        LOG_OK("Pair %i: %s", i, rows[i].c_str());
    }

    std::cout << "Rotation angles :\n" << rot.format(CleanFmt) << std::endl;
}

void ProjectManager::triangulate(ProjectData &data)
{
    LOG_DBG("MAKE BUTTON DISABLED");

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
    auto nbPts  = Wf.cols();
    Mat4X pts3D;
    pts3D.setZero(4,nbPts);

    for (auto p = 0; p < nbPts; ++p) {
        std::vector<Vec2> x;
        std::vector<Mat34> P;
        for (auto c = 0; c < nbCams; c++) {
            if (Wf(3*c + 2,p) == 1.0) { // there is a point
                x.emplace_back(Wf.block(3*c,p,2,1));
                P.emplace_back(Ps[c]);
            }
        }
        pts3D.col(p) = utils::triangulate(x,P);
        pts3D.col(p) /= pts3D(3,p);
    }

    CommandManager::get()->executeCommand(
                new CommandSetProperty(&data,P3D_ID_TYPE(p3dData_pts3DSparse),pts3D,true)
                );
    LOG_OK("Triangulated %i points", data.getPts3DSparse().cols());
}

void ProjectManager::triangulateStereo(ProjectData &data)
{
    auto pairIdx = 0;
    auto imPair = data.imagePair(pairIdx);
    if (!imPair) return;
    if (!imPair->hasDisparityMap()) return;

    auto rho = imPair->getRho();
    if (utils::floatEq(rho,0.0)) return;

    float cosRho = cos(rho);
    float sinRho = sin(rho);

    auto dispValues = data.imagePair(pairIdx)->getDisparityMap().clone();
    if (dispValues.type() != CV_32F) {
        LOG_ERR("Disparity map type must be CV_32F");
        return;
    }
    dispValues = dispValues / 16.0;

    std::vector<Vec3f> pts3D;
    std::vector<Vec3f> colors;

//#ifdef WITH_OPENMP
//    omp_set_num_threads(utils::nbAvailableThreads());
//    #pragma omp parallel for
//#endif

    cv::Mat I = data.imagePair(pairIdx)->getRectifiedImageL();
    if (I.type() != CV_8UC3) LOG_DBG("Rectified image has wrong type: %i", I.type());

    for (int u = 0; u < dispValues.cols; ++u) {
        for (int v = 0; v < dispValues.rows; ++v) {
            const float d = dispValues.at<float>(v,u);
            if (d * 0.0 != 0.0) continue; // check for NaN

            const float q1x = u;
            const float q1y = v;
            const float q2x = float(u) + d;

            const float q1z = (q1x * cosRho - q2x) / sinRho;

//            #pragma omp critical
            cv::Vec3b c = I.at<cv::Vec3b>(v,u);
            if (c != cv::Vec3b(0,0,0))
            {
                pts3D.emplace_back(Vec3f(q1x,q1y,q1z));
                colors.emplace_back(Vec3f(c.val[0]/ 255.f,c.val[1]/ 255.f,c.val[2]/ 255.f));
            }
        }
    }

    Mat3Xf result;
    Mat3Xf colorsMat;
    utils::convert(pts3D,result);
    utils::convert(colors,colorsMat);

    CommandManager::get()->executeCommand(
                new CommandSetProperty(&data,P3D_ID_TYPE(p3dData_pts3DDense),result,true)
                );
    CommandManager::get()->executeCommand(
                new CommandSetProperty(&data,P3D_ID_TYPE(p3dData_pts3DDenseColors),colorsMat,true)
                );

    LOG_OK("Triangulated (stereo) %i points", pts3D.size());
}

void ProjectManager::triangulateDense(ProjectData &data)
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
                const double d = static_cast<double>(dispValues.at<float>(v,u));
                if (d * 0.0 != 0.0) continue; // check for NaN

                Vec3 pl = Tli * Vec3(u, v, 1.0);
                Vec3 pr = Tri * Vec3(double(u)+d, v, 1.0);

                if (pl(0) < 0) continue;
                if (pl(1) < 0) continue;
                if (pr(0) < 0) continue;
                if (pr(1) < 0) continue;

                //if (pl(0) > im1.width()) continue;
                //if (pl(1) > im1.height()) continue;
                //if (pr(0) > im2.width()) continue;
                //if (pr(1) > im2.height()) continue;

                matches.push_back({pl.topRows(2),pr.topRows(2)});
            }
        }

        auto Ps = data.getCameraMatrices();
        std::vector<Mat34> P;
        P.push_back(Ps[pairIdx]);
        P.push_back(Ps[pairIdx+1]);

        int nbGoodPoints = 0;
//    #ifdef WITH_OPENMP
//        omp_set_num_threads(utils::nbAvailableThreads());
//        #pragma omp parallel for
//    #endif
        for (auto p = 0; p < matches.size(); ++p) {
            Vec4 pt = utils::triangulate(matches[p],P);
            pt /= pt(3);

            // check consistensy
            double err = 0.0;
            for (auto c = 0; c < P.size(); ++c) {
                Vec3 q1 = P[c] * pt;
                q1 /= q1(2);
                const auto & xr = q1(0);
                const auto & yr = q1(1);
                const auto & x = matches[p][c](0);
                const auto & y = matches[p][c](1);
                err += (xr - x)*(xr - x) + (yr - y)*(yr - y);
            }
            bool inlier = err < 2*2;
            //#pragma omp critical
            if (inlier) {
                pts3D.push_back(pt.topRows(3));
                nbGoodPoints++;
            }
        }
        LOG_INFO("Good points: %0.2f %%", 100.0f * nbGoodPoints / float(matches.size()));
    }

    Mat3Xf result;
    utils::convert(pts3D,result);

    CommandManager::get()->executeCommand(
                new CommandSetProperty(&data,P3D_ID_TYPE(p3dData_pts3DDense),result,true)
                );

    LOG_OK("Triangulated %i points", pts3D.size());
}

void ProjectManager::bundleAdjustment(ProjectData &data)
{
    BundleProblem p;
    p.X = data.getPts3DSparse();
    p.W = data.getMeasurementMatrixFull();

    if (p.X.cols() != p.W.cols()) {
        LOG_ERR("Measurement matrix doesn't correspond to 3D points");
        return;
    }

    data.getCamerasIntrinsics(&p.cam);
    data.getCamerasExtrinsics(&p.R,&p.t);

    LOG_OK("Bundle adjustement started...");
    const auto nbCams = p.R.size();
    {
        BundleParams params(nbCams);
        params.setConstAll();
        params.setVaryingPts();
        BundleAdjustment ba;
        ba.run(p,params);
    }

    {
        BundleParams params(nbCams);
        params.setConstAll();
        params.setVaryingAllCams(p3dBundleParam_R);
        params.setVaryingAllCams(p3dBundleParam_t);
        params.setConstAllParams({0});
        BundleAdjustment ba;
        ba.run(p,params);
    }

    {
        BundleParams params(nbCams);
        params.setConstAll();
        params.setVaryingPts();
        BundleAdjustment ba;
        ba.run(p,params);
    }

    data.setPts3DSparse(p.X);
    data.setCamerasIntrinsics(p.cam);
    data.setCamerasExtrinsics(p.R,p.t);

    LOG_OK("Bundle adjustement: done");
}

void ProjectManager::exportPLY(ProjectData &data)
{
    auto X = data.getPts3DSparse();
    if (X.cols() == 0) {
        LOG_ERR("No reconstructed points...");
        return;
    }

    utils::exportToPly(X,"point_cloud.ply");

    LOG_OK("Exported point cloud: %i points", X.cols());

}

entt::meta_any ProjectManager::getSetting(const p3dSetting &name) {
    auto data = entt::resolve<ProjectSettings>().data(P3D_ID_TYPE(name));
    if (!data) return entt::meta_any{nullptr};
    return data.get(m_settings);
}

void ProjectManager::setSetting(const p3dSetting &id, const entt::meta_any &value) {
    CommandManager::get()->executeCommand(new CommandSetProperty(&m_settings,P3D_ID_TYPE(id),value));
}
