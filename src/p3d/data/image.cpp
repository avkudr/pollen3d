#include "image.h"

#include "p3d/core/core.h"
#include "p3d/core/utils.h"

int dummyImage_ = Image::initMeta();

int Image::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: Image" << std::endl;
        entt::meta<Image>()
            .type("Image"_hs)
            .data<&Image::setPath,&Image::getPath>(P3D_ID_TYPE(p3dImage_path))
            .data<&Image::setCamera,&Image::getCamera>(P3D_ID_TYPE(p3dImage_camera));
        SERIALIZE_TYPE_VECS(Image,"vector_Image"_hs);
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
        LOG_ERR("* Failed to load %s", path.c_str());
        _path = "";
    } else {
        LOG_INFO("* Loaded: %s", path.c_str());
        _path = path;
        _imageCV = im.clone();
    }
}

void Image::setKeyPoints(std::vector<cv::KeyPoint> kpts){
    _kpts.clear();
    _kpts = kpts;
}

/*
void Image::updatesssProperties()
{
    _kptsMat = cv::Mat::ones(3,(int)_kpts.size(),cv::DataType<double>::type);
    for( int i = 0; i < _kpts.size() ; i++)
    {
        _kptsMat.at<double>(0,i) = _kpts[i].pt.x;
        _kptsMat.at<double>(1,i) = _kpts[i].pt.y;
    }

    _properties.clear();
    _properties.push_back(new CVMatProperty(&_kptsMat, "Features"));
    _properties.push_back(new CVMatProperty(&K, "Intrinsic"));
    _properties.push_back(new CVMatProperty(&P, "Camera matrix"));
}
*/
