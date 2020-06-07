#pragma once

#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/image/feature_extraction.h"
#include "p3d/logger.h"
#include "p3d/serialization.h"
#include "p3d/stereo/neighbor.h"
#include "p3d/utils.h"

namespace p3d
{
enum P3D_API p3dImage_ {
    p3dImage_path = 0,
    p3dImage_descriptors = 1,
    p3dImage_keypoints = 2,
    p3dImage_camera = 3,
    p3dImage_translation = 4,
    p3dImage_featExtractionPars,
    p3dImage_rotation,
};

class P3D_API Image : public Serializable<Image>
{
public:
    static int initMeta();
    static const char *classNameStatic() { return "Image"; }

    Image() {}
    Image(cv::Mat im);
    Image(const std::string &path);
    ~Image() {}

    // ***** interface

    const cv::Mat &cvMat() const { return _imageCV; }
    int width() const { return _imageCV.cols; }
    int height() const { return _imageCV.rows; }
    cv::Size size() const { return _imageCV.size(); }

    operator cv::Mat() { return _imageCV; }
    operator cv::_InputArray() { return _imageCV; }

    void setPath(const std::string &p) { _path = p; }
    std::string getPath() const { return _path; }
    std::string name() const { return utils::baseNameFromPath(_path); }

    bool isValid() { return _path != "" && !_imageCV.empty(); }

    // ***** features

    bool hasFeatures() { return _kpts.size() > 0; }
    int nbFeatures() const { return _kpts.size(); }
    int getNbFeatures() const { return nbFeatures(); }  // refactor delete
    const std::vector<cv::KeyPoint> &getKeyPoints() const { return _kpts; }
    void setKeyPoints(std::vector<cv::KeyPoint> kpts);

    const cv::Mat &getDescriptors() const { return _desc; }
    void setDescriptors(cv::Mat desc) { _desc = desc.clone(); }

    void writeAdditional(cv::FileStorage &fs) override;
    void readAdditional(const cv::FileNode &node) override;

    // ***** camera parameters

    Mat34 getCameraMatrix() const
    {
        Mat34 P;
        P.setZero();
        const Mat2 A = getCamera().getA();
        const Mat23 R = getRotationAsMatrix().topRows(2);

        P.setZero();
        P.topLeftCorner(2, 3) = A * R;
        P.topRightCorner(2, 1) = getTranslation();
        P(2, 3) = 1.0;
        return P;
    }

    bool hasCamera() { return !m_camera.getA().isApprox(Mat2::Identity()); }
    void setCamera(const AffineCamera &c) { m_camera = c; }
    const AffineCamera &getCamera() const { return m_camera; }

    void setTranslation(const Vec2 &t) { m_t = t; }
    Vec2 getTranslation() const { return m_t; }

    void setRotation(const Vec3 &rvec) { m_rvec = rvec; }
    Vec3 getRotation() const { return m_rvec; }
    Vec3 setRotationFromMatrix(const Mat3 &R)
    {
        utils::EulerZYXfromR(R, m_rvec[0], m_rvec[1], m_rvec[2]);
    }
    Mat3 getRotationAsMatrix() const
    {
        return utils::RfromEulerZYX(m_rvec[0], m_rvec[1], m_rvec[2]);
    }

    FeatExtractionPars *featExtractionPars();
    const FeatExtractionPars &getFeatExtractionPars() const;
    void setFeatExtractionPars(const FeatExtractionPars &featExtractionPars);

private:
    std::string  EMPTY_PATH{"<not found>"};
    std::string  _path{"<not found>"};
    cv::Mat      _imageCV{};
    AffineCamera m_camera{};
    Vec2 m_t{0, 0};
    Vec3 m_rvec{0, 0, 0};

    std::vector<cv::KeyPoint> _kpts{};
    cv::Mat                   _desc{};

    FeatExtractionPars m_featExtractionPars{};
};
}  // namespace p3d
