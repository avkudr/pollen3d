#pragma once

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/logger.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

namespace p3d
{
enum p3dImage_
{
    p3dImage_path = 0,
    p3dImage_descriptors = 1,
    p3dImage_keypoints = 2,
    p3dImage_camera = 3,
    p3dImage_translation = 4,
};

class Image : public Serializable<Image>{
public:
    static int initMeta();

    Image(){}
    Image(cv::Mat im);
    Image(const std::string &path);
    ~Image(){}

    // Interface
    cv::Mat getREFACTOR(){ return _imageCV; } //original, withInterestPoints...
    const cv::Mat & cvMat() const { return _imageCV; } //original, withInterestPoints...
    int width() const { return _imageCV.cols; }
    int height() const { return _imageCV.rows; }
    cv::Size size() const { return _imageCV.size(); }

    operator cv::Mat(){ return _imageCV; }
    operator cv::_InputArray(){ return _imageCV; }

    void setPath(const std::string & p){ _path = p; }
    std::string getPath() const { return _path; }
    std::string name() const { return utils::baseNameFromPath(_path); }

    bool isValid(){
        return _path != "" && !_imageCV.empty();
    }

    // Features

    bool hasFeatures(){ return _kpts.size() > 0; }
    int getNbFeatures() const { return _kpts.size();}
    const std::vector<cv::KeyPoint> &getKeyPoints() { return _kpts; }
    void setKeyPoints(std::vector<cv::KeyPoint> kpts);

    const cv::Mat & getDescriptors() const { return _desc; }
    void setDescriptors(cv::Mat desc) { _desc = desc.clone(); }

    void writeAdditional(cv::FileStorage &fs) override {
        fs << "im_p" + std::to_string(int(p3dImage_descriptors)) << _desc;
        fs << "im_p" + std::to_string(int(p3dImage_keypoints)) << _kpts;
    }

    void readAdditional(const cv::FileNode &node) override {
        if (_path != EMPTY_PATH) {
            _imageCV = cv::imread(_path);
            if(! _imageCV.data){
                LOG_INFO("No such image: %s", _path.c_str());
                return;
            }
        }

        node["im_p" + std::to_string(int(p3dImage_descriptors))] >> _desc;
        node["im_p" + std::to_string(int(p3dImage_keypoints))  ] >> _kpts;
    }

    inline bool operator==(const Image& i) const{
        if ( _path != i._path ) return false;
        if (!(getCamera() == i.getCamera())) return false;
        if (!utils::floatEq(m_t[0],i.getTranslation()[0])) return false;
        if (!utils::floatEq(m_t[1],i.getTranslation()[1])) return false;
        return true;
    }

    // ***** camera parameters

    bool hasCamera() { return !m_camera.getA().isApprox(Mat2::Identity()); }
    void setCamera(const AffineCamera & c) { m_camera = c; }
    const AffineCamera & getCamera() const { return m_camera; }

    void setTranslation(const Vec2 & t) { m_t = t; }
    Vec2 getTranslation() const { return m_t; }

private:

    std::string EMPTY_PATH = "<not found>";
    std::string _path = "<not found>";
    cv::Mat _imageCV;
    AffineCamera m_camera{};
    Vec2 m_t{0,0};

public:
    std::vector<cv::KeyPoint> _kpts{};
    cv::Mat _desc;
};
}  // namespace p3d
