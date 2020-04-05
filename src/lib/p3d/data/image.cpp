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
            .type("Image"_hs)
            .data<&Image::setPath, &Image::getPath>(P3D_ID_TYPE(p3dImage_path))
            .data<&Image::setCamera, &Image::getCamera>(
                P3D_ID_TYPE(p3dImage_camera))
            .data<&Image::setTranslation, &Image::getTranslation>(
                P3D_ID_TYPE(p3dImage_translation));

        SERIALIZE_TYPE_VECS(Image, "vector_Image"_hs);
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

bool Image::operator==(const Image &i) const
{
    if (_path != i._path) return false;
    if (!(getCamera() == i.getCamera())) return false;
    if (!utils::floatEq(m_t[0], i.getTranslation()[0])) return false;
    if (!utils::floatEq(m_t[1], i.getTranslation()[1])) return false;
    return true;
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
