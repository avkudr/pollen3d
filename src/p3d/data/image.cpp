#include "image.h"

#include "p3d/core/core.h"
#include "p3d/core/utils.h"

int dummyImage_ = Image::initMeta();

int Image::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: Image" << std::endl;
        meta::reflect<Image>(p3d_hashStr("Image"))
            .data<&Image::setPath,&Image::getPath>(p3d_hash(p3dImage_path));
        SERIALIZE_TYPE_VECS(Image,"vector_Image");
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

cv::Mat Image::getFeaturePositions() const
{
    int nbPts = (int) _kpts.size();

    cv::Mat W = cv::Mat::zeros(2,nbPts,cv::DataType<double>::type);

    for (int i = 0; i < nbPts; i++){
        W.at<double>(0,i) = _kpts[i].pt.x;
        W.at<double>(1,i) = _kpts[i].pt.y;
    }

    return W;
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
