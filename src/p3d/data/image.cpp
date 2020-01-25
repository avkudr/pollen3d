#include "image.h"

#include "p3d/core/core.h"
#include "p3d/core/utils.h"

Image::Image(cv::Mat im)
{
    _imageCV = im;
    init();
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
        init();
    }
}

cv::Mat Image::getImageWithFeatures()
{
    cv::Mat img;
    _imageCV.copyTo(img);
    if ( getFeaturesNb() != 0){
        cv::cvtColor(img,img,CV_BGR2GRAY);
        std::vector<cv::KeyPoint> kpts = getKeyPoints();
        cv::drawKeypoints(img, getKeyPoints(), img, cv::Scalar(255,0,0));
//        for ( auto i = 0; i < kpts.size(); i++){
//            cv::circle( img, kpts[i], 2.0, Scalar( 0, 0, 255 ), 1, 8 );
//        }
    }
    return img;
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

void Image::init()
{
    static bool firstCall = true;
    if (firstCall) {
        meta::reflect<Image>(p3d_hashStr("Image"))
            .data<&Image::setPath,&Image::getPath>(p3d_hash(p3dImage_path));

        //.data<&Image::setDescriptors,&Image::getDescriptors>(p3d_hash(p3dImage_descriptors));

        firstCall = false;
    }
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
