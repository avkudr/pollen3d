#pragma once

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/core.h"
#include "p3d/console_logger.h"
#include "p3d/core/serialization.h"
#include "p3d/core/imageproperties.h"

using std::vector;
using std::string;

enum p3dImage_
{
    p3dImage_path = 0,
    p3dImage_descriptors = 1,
    p3dImage_keypoints = 1,
};

class Image : public Serializable<Image>{
public:
    Image(){ init(); }
    Image(cv::Mat im);
    Image(const std::string &path);
    ~Image(){}

    // Interface
    cv::Mat get(){ return _imageCV; } //original, withInterestPoints...
    const cv::Mat & cvMat() const { return _imageCV; } //original, withInterestPoints...

    operator cv::Mat(){ return _imageCV; }
    operator cv::_InputArray(){ return _imageCV; }

    void setPath(const std::string & p){ _path = p; }
    std::string getPath() const { return _path; }
    std::string name() const { return utils::baseNameFromPath(_path); }

    bool isValid(){
        return _path != "" && !_imageCV.empty();
    }
    cv::Mat getImageOriginal(){ return _imageCV; } //original, withInterestPoints...
    cv::Mat getImageWithFeatures();

    // Features

    bool isFeaturesExtracted(){ return _kpts.size() > 0; }
    int getFeaturesNb(){ return (int)_kpts.size();}
    cv::Mat getFeaturePositions() const;
    const vector<cv::KeyPoint> & getKeyPoints(){ return _kpts;}
    void setKeyPoints(std::vector<cv::KeyPoint> kpts);

    const cv::Mat & getDescriptors() const { return _desc; }
    void setDescriptors(cv::Mat desc) { _desc = desc.clone(); }

    vector<Property*> _properties; 

    void writeAdditional(cv::FileStorage &fs) override {
        fs << "im_p" + std::to_string(int(p3dImage_descriptors)) << _desc;
        fs << "im_p" + std::to_string(int(p3dImage_keypoints)) << _kpts;
    }

    void readAdditional(const cv::FileNode &node) override {
        if (_path != "noName") {
            _imageCV = cv::imread(_path);
            if(! _imageCV.data){
                LOG_INFO("No such image: %s", _path.c_str());
                return;
            }
        }

        node["im_p" + std::to_string(int(p3dImage_descriptors))] >> _desc;
        node["im_p" + std::to_string(int(p3dImage_keypoints))  ] >> _kpts;
    }

private:
    void init();

    std::string _path = "noName";
    cv::Mat _imageCV;

public:
    std::vector<cv::KeyPoint> _kpts;
    cv::Mat _desc;

    cv::Mat K;
    cv::Mat P;
    cv::Mat getIntrinsic() const {return K.clone();}
    cv::Mat getCameraMatrix() const {return P.clone();}
};

//These write and read functions must be defined for the serialization in FileStorage to work
[[maybe_unused]]
static void write(cv::FileStorage& fs, const std::string&, const Image& x)
{
    fs << "{";
    const_cast<Image&>(x).write(fs);
    fs << "}";
}

[[maybe_unused]]
static void read(const cv::FileNode& node, Image& x, const Image& default_value = Image()){
    if(node.empty())
        x = default_value;
    else {
        x.read(node);
    }
}

