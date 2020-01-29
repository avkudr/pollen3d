#include "project_manager.h"

#include <omp.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

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
        settings.write(fs);
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
            settings.read(fs["ProjectSettings"]);
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

void ProjectManager::extractFeatures(ProjectData &imList, std::vector<int> imIds)
{
    if (imList.nbImages() == 0) return;
    if (imIds.empty())
        for (int i = 0; i < imList.nbImages(); ++i) imIds.push_back(i);

    int nbImgs = imIds.size();
#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(nbImgs,utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (int i = 0; i < nbImgs; i++) {
        Image * im = imList.image(imIds[i]);
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

void ProjectManager::matchFeatures(ProjectData &imList, std::vector<int> imPairsIds)
{
    if (imList.nbImagePairs() == 0) return;
    if (imPairsIds.empty())
        for (int i = 0; i < imList.nbImagePairs(); ++i) imPairsIds.push_back(i);

    int nbPairs = imPairsIds.size();

    float filterCoef = getSetting(p3dSetting_matcherFilterCoef).cast<float>();

#ifdef WITH_OPENMP
    omp_set_num_threads(std::min(int(imList.nbImagePairs()),utils::nbAvailableThreads()));
    #pragma omp parallel for
#endif
    for (int i = 0; i < nbPairs; i++) {
        auto imPair = imList.imagePair(i);
        if (!imPair || !imPair->isValid()) continue;

        auto imIdxL = imPair->imL();
        auto imIdxR = imPair->imR();
        auto imL = imList.image(imIdxL);
        auto imR = imList.image(imIdxR);

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

        LOG_OK("Pair %i, matched %i features", i, matchesPair.size());
    }
}

meta::any ProjectManager::getSetting(const p3dSetting &name) {
    auto data = meta::resolve<ProjectSettings>().data(p3d_hash(name));
    if (!data) return meta::any{nullptr};
    return data.get(settings);
}

void ProjectManager::setSetting(const p3dSetting &name, meta::any value) {
    CommandManager::get()->executeCommand(new CommandSetProperty(&settings,name,value));
}
