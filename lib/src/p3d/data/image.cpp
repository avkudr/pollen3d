#include "image.h"

#include "p3d/core.h"
#include "p3d/utils.h"

using namespace p3d;

int dummyImage_ = Image::initMeta();

int Image::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        // reflecting: Image
        entt::meta<Image>()
            .alias(p3d::alias(classNameStatic()))
            .data<&Image::setPath, &Image::getPath>(P3D_ID_TYPE(p3dImage_path))
            .data<&Image::setCamera, &Image::getCamera>(P3D_ID_TYPE(p3dImage_camera))
            .data<&Image::setTranslation, &Image::getTranslation>(
                P3D_ID_TYPE(p3dImage_translation))
            .data<&Image::setFeatExtractionPars, &Image::getFeatExtractionPars>(
                P3D_ID_TYPE(p3dImage_featExtractionPars));

        SERIALIZE_TYPE_VECS(Image);
        firstCall = false;
    }
    return 0;
}


Image::Image(cv::Mat im)
{
    _imageCV = im;
}

Image::Image(const std::string & path)
{
    cv::Mat im = cv::imread(path);
    if (im.empty()) {
        LOG_ERR("- failed to load %s", path.c_str());
        _path = "";
    } else {
        LOG_INFO("- loaded: %s", path.c_str());
        _path = path;
        _imageCV = im.clone();
    }
}

void Image::setKeyPoints(std::vector<cv::KeyPoint> kpts){
    _kpts.clear();
    _kpts = kpts;
}

void Image::writeAdditional(cv::FileStorage &fs)
{
    fs << "im_p" + std::to_string(int(p3dImage_descriptors)) << _desc;
    fs << "im_p" + std::to_string(int(p3dImage_keypoints)) << _kpts;
}

void Image::readAdditional(const cv::FileNode &node)
{
    if (_path != EMPTY_PATH) {
        _imageCV = cv::imread(_path);
        if (!_imageCV.data) {
            LOG_INFO("no such image: %s", _path.c_str());
            return;
        }
    }

    node["im_p" + std::to_string(int(p3dImage_descriptors))] >> _desc;
    node["im_p" + std::to_string(int(p3dImage_keypoints))] >> _kpts;
}

FeatExtractionPars *Image::featExtractionPars()
{
    return &m_featExtractionPars;
}

const FeatExtractionPars &Image::getFeatExtractionPars() const
{
    return m_featExtractionPars;
}

void Image::setFeatExtractionPars(const FeatExtractionPars &featExtractionPars)
{
    m_featExtractionPars = featExtractionPars;
}
