#include "project_manager.h"

#include <omp.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <map>

#include "tinyfiledialogs/tinyfiledialogs.h"

#include "p3d/console_logger.h"
#include "p3d/data/image.h"
#include "p3d/core/utils.h"
#include "p3d/commands.h"

#define P3D_PROJECT_EXTENSION ".yml.gz"

ProjectManager *ProjectManager::m_instance = nullptr;

std::vector<std::string> ProjectManager::loadImagesDialog()
{
    std::vector<std::string> out;
    char const * lTheOpenFileName;
    char const * lFilterPatterns[] = { "*.jpg","*.jpeg", "*.png", "*.tif", "*.tiff" };

    lTheOpenFileName = tinyfd_openFileDialog(
                         "Load images...",
                         "",
                         5,
                         lFilterPatterns,
                         nullptr,
                         1);
    if (!lTheOpenFileName) return out;

    std::string allFiles(lTheOpenFileName);
    out = utils::split(lTheOpenFileName,"|");
    return out;
}

std::string ProjectManager::openProjectDialog()
{
    char const * lTheOpenFileName;
    std::string ext = "*" + std::string(P3D_PROJECT_EXTENSION);
    char const * lFilterPatterns[] = { ext.c_str() };

    lTheOpenFileName = tinyfd_openFileDialog(
                         "Load images...",
                         "",
                         1,
                         lFilterPatterns,
                         nullptr,
                         0);
    if (!lTheOpenFileName) return "";

    std::string projectFile(lTheOpenFileName);
    return projectFile;
}

std::string ProjectManager::saveProjectDialog()
{
    char const * saveFilePath;
    std::string ext = "*" + std::string(P3D_PROJECT_EXTENSION);
    char const * saveFilePattern[] = { ext.c_str() };

    saveFilePath = tinyfd_saveFileDialog (
            "Save project as..." ,
            "" ,
            1, // nb files to save
            saveFilePattern,
            "Pollen3D project") ;

    if (!saveFilePath) return "";

    std::string out(saveFilePath);
    if (!utils::endsWith(out,P3D_PROJECT_EXTENSION)) out += P3D_PROJECT_EXTENSION;
    return out;
}

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

    list->setImageList(imgs);
}

void ProjectManager::saveProject(ProjectData *data, std::string path) {

    if (!data) return;
    if (path == "" && data->getProjectPath() == ""){
        path = saveProjectDialog();
    }
    if ( path == "") {
        LOG_ERR("Error while saving project");
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

void ProjectManager::openProject(ProjectData *data, std::string path)
{
    if (!data) return;
    if (path == "") return;
    if (!utils::endsWith(path,P3D_PROJECT_EXTENSION)) return;

    LOG_OK( "Loading %s", path.c_str() );

    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            m_settings.read(fs["ProjectSettings"]);
            data->read(fs["ProjectData"]);
            LOG_OK( "Done" );
            fs.release();
        } else {
            LOG_ERR( "Can't write to %s", path.c_str() );
        }
    } catch (const cv::Exception & e) {
        LOG_ERR("cvErr:%s",e.what());
    }
}

void ProjectManager::extractFeatures(ProjectData &data, std::vector<int> imIds)
{
    if (data.nbImages() == 0) return;
    if (imIds.empty())
        for (int i = 0; i < data.nbImages(); ++i) imIds.push_back(i);

    int nbImgs = imIds.size();
#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(nbImgs,utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (int i = 0; i < nbImgs; i++) {
        Image * im = data.image(imIds[i]);
        if (!im) continue;
        if (im->cvMat().empty()){
            LOG_ERR("Features cannot be extracted: no image loaded");
            continue;
        }

        std::vector<cv::KeyPoint> kpts;
        cv::Mat desc;

        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB,0,3,0.001f );
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

        LOG_OK("Pair %i, matched %i features", imPairsIds[i], matchesPair.size());
    }
}

void ProjectManager::findFundamentalMatrix(ProjectData &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(data.nbImagePairs()),utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (size_t idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];
        std::vector<Vec2> ptsL, ptsR;
        data.getPairwiseMatches(i, ptsL, ptsR);
        if (ptsL.empty() || ptsR.empty()) {
            LOG_OK("Pair %i has no matches", i);
            continue;
        }

        Mat3 F = FundMatAlgorithms::findFundMatCeres(ptsL,ptsR);
        data.imagePair(i)->setFundMat(F);

        LOG_OK("Pair %i, estimated F (ceres)", i);
    }
}

void ProjectManager::rectifyImagePairs(ProjectData &data, std::vector<int> imPairsIds)
{
    if (data.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < data.nbImagePairs(); ++i) imPairsIds.push_back(i);

//#ifdef WITH_OPENMP
//    omp_set_num_threads(std::min(int(data.nbImagePairs()),utils::nbAvailableThreads()));
//    #pragma omp parallel for
//#endif
    for (size_t idx = 0; idx < imPairsIds.size(); idx++) {
        auto i = imPairsIds[idx];

        auto imPair = data.imagePair(i);
        if (!imPair) continue;
        if (!imPair->hasF()) {
            LOG_ERR("Pair %i, no fundamental matrix", i);
            continue;
        }

        auto imL = data.image(imPair->imL());
        auto imR = data.image(imPair->imR());

        if (!imL || !imR) continue;

        // we can get epipolar lines for any point as
        // they are all parallel for affine camera
        std::vector<Vec3> epiL,epiR;
        imPair->getEpilinesLeft({Vec2(100,100)},epiL);
        imPair->getEpilinesRight({Vec2(100,100)},epiR);

        double angleL = std::atan(-epiL[0](0)/epiL[0](1));
        double angleR = std::atan(-epiR[0](0)/epiR[0](1));

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

entt::meta_any ProjectManager::getSetting(const p3dSetting &name) {
    auto data = entt::resolve<ProjectSettings>().data(P3D_ID_TYPE(name));
    if (!data) return entt::meta_any{nullptr};
    return data.get(m_settings);
}

void ProjectManager::setSetting(const p3dSetting &id, const entt::meta_any &value) {
    CommandManager::get()->executeCommand(new CommandSetProperty(&m_settings,P3D_ID_TYPE(id),value));
}
