#ifndef IMAGE_PAIR_H
#define IMAGE_PAIR_H

#include <vector>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/core.h"
#include "p3d/data/image.h"
#include "p3d/core/fundmat.h"
#include "p3d/core/serialization.h"

using std::vector;
using std::string;

enum p3dDense_
{
    p3dDense_DispMap = 0,
    p3dDense_DispMethod,
    p3dDense_DispLowerBound,
    p3dDense_DispUpperBound,
    p3dDense_DispBlockSize,

    p3dDense_DispFilterNewValue, // The disparity value used to paint-off the speckles
    p3dDense_DispFilterMaxSpeckleSize, // The maximum speckle size to consider it a speckle. Larger blobs are not affected by the algorithm
    p3dDense_DispFilterMaxDiff, // Maximum difference between neighbor disparity pixels to put them into the same blob. Note that since StereoBM,
    //StereoSGBM and may be other algorithms return a fixed-point disparity map, where disparity values are multiplied by 16,
    //this scale factor should be taken into account when specifying this parameter value.

    p3dDense_BilateralD, // The disparity value used to paint-off the speckles
    p3dDense_BilateralSigmaColor, // The maximum speckle size to consider it a speckle. Larger blobs are not affected by the algorithm
    p3dDense_BilateralSigmaSpace, // Maximum difference between neighbor disparity pixels to put them into the same blob. Note that since StereoBM,
    //StereoSGBM and may be other algorithms return a fixed-point disparity map, where disparity values are multiplied by 16,
    //this scale factor should be taken into account when specifying this parameter value.


};

class DenseMatching : public Serializable<DenseMatching>{
public:
    enum DenseMatchingMethod_{
        DenseMatchingMethod_SGBM      = cv::StereoSGBM::MODE_SGBM,          ///< Perform parabolic interpolation on the table
        DenseMatchingMethod_HH        = cv::StereoSGBM::MODE_HH,          ///< Perform linear interpolation on the table
        DenseMatchingMethod_SGBM_3WAY = cv::StereoSGBM::MODE_SGBM_3WAY           ///< Perform parabolic interpolation on the table
    };

    static int initMeta();

    cv::Mat dispMap;

    int dispMethod{DenseMatchingMethod_SGBM};
    int dispLowerBound{-1};
    int dispUpperBound{2};
    int dispBlockSize{9};

    int dispFilterNewValue{0};
    int dispFilterMaxSpeckleSize{260};
    int dispFilterMaxDiff{10};

    int bilateralD{9};
    int bilateralSigmaColor{180};
    int bilateralSigmaSpace{180};

};

enum p3dMatch_
{
    p3dMatch_iPtL = 100,
    p3dMatch_iPtR = 101,
    p3dMatch_distance = 102,
};

class Match : public Serializable<Match>{
public:
    static int initMeta();

    Match(){
    }
    Match(int idxPtImageL, int idxPtImageR, float dist = 0.0f) :
        iPtL(idxPtImageL), iPtR(idxPtImageR), distance(dist)
    {
    }
    ~Match(){}

    int iPtL{0};
    int iPtR{0};
    float distance = 0;

    inline bool operator==(const Match& a) const{
        return iPtL == a.iPtL && iPtR == a.iPtR;
    }
};

enum p3dImagePair_
{
    p3dImagePair_matches = 0,
    p3dImagePair_imL     = 1,
    p3dImagePair_imR     = 2,
    p3dImagePair_fundMat = 3,
    p3dImagePair_hasF    = 4,
    p3dImagePair_rectifyingTransformLeft,
    p3dImagePair_rectifyingTransformRight,
    p3dImagePair_rectifiedImageLeft,
    p3dImagePair_rectifiedImageRight,
    p3dImagePair_disparityMap,
    p3dImagePair_Theta1, // theta_1
    p3dImagePair_Rho   , // rho
    p3dImagePair_Theta2, // theta_2
    p3dImagePair_denseMatching,
};

class ImagePair : public Serializable<ImagePair>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    ImagePair(int imL = -1, int imR = -1);
    ~ImagePair() override;

    bool operator==(const ImagePair& i) const;

    bool isValid() { return _imL >= 0 && _imR >= 0;}
    bool hasMatches() { return getNbMatches() > 0; }
    int getNbMatches() { return int(m_matches.size()); }
    const std::vector<Match> & getMatches() const { return m_matches; }
    void getMatchesAsMap(std::map<int,int> & map) const;
    void setMatches(const std::vector<Match> & matches) { m_matches = matches; }

    cv::Mat getOneStereoImage(cv::Mat im1, cv::Mat im2);

    int imL() { return _imL; }
    int imR() { return _imR; }
    void setLeftImage(int imL) { _imL = imL; }
    void setRightImage(int imR) { _imR = imR; }

    // ***** Epipolar geometry

    bool hasF() const;
    const Mat3 & getFundMat() const { return F; }
    void setFundMat(const Mat3 & f) { F = f; }
    void getEpilines(const std::vector<Vec2> & pts, std::vector<Vec3> & epilines, bool transpose) const;
    void getEpilinesLeft(const std::vector<Vec2> & ptsR, std::vector<Vec3> & epilinesL) const { getEpilines(ptsR, epilinesL, true);}
    void getEpilinesRight(const std::vector<Vec2> & ptsL, std::vector<Vec3> & epilinesR) const { getEpilines(ptsL, epilinesR, false); }

    std::vector<cv::Point2d> getInliersLeftImageREFACTOR() { return getInliersREFACTOR(0); }
    std::vector<cv::Point2d> getInliersRightImageREFACTOR() { return getInliersREFACTOR(1); }
    std::vector<cv::Point2d> getInliersREFACTOR(int imageIdx) {return std::vector<cv::Point2d>();/*_matcher->getInliers(imageIdx);*/}

    // ***** Rectification

    bool isRectified() { return !m_imLrectified.empty() && !m_imRrectified.empty(); }

    const cv::Mat & getRectifiedImageL() const { return m_imLrectified; }
    const cv::Mat & getRectifiedImageR() const { return m_imRrectified; }

    void setRectifiedImageL(const cv::Mat & im) { m_imLrectified = im; }
    void setRectifiedImageR(const cv::Mat & im) { m_imRrectified = im; }

    Mat3 getRectifyingTransformL() const { return m_rectifyingTransformL;}
    Mat3 getRectifyingTransformR() const { return m_rectifyingTransformR;}
    void setRectifyingTransformL(const Mat3 & m) { m_rectifyingTransformL = m;}
    void setRectifyingTransformR(const Mat3 & m) { m_rectifyingTransformR = m;}

    // ***** Dense matching

    bool hasDisparityMap() { return !m_disparityMap.empty(); }

    const cv::Mat & getDisparityMap() const { return m_disparityMap; }
    void setDisparityMap(const cv::Mat & im) { m_disparityMap = im; }
    cv::Mat getDisparityMapPlot() const;

    // **** rotation parameters

    void setTheta1(double a) { m_theta1 = a; }
    void setRho   (double a) { m_rho    = a; }
    void setTheta2(double a) { m_theta2 = a; }

    bool  hasRrelative() { return !utils::floatEq(m_rho,0.0); }
    const Mat3 getRrelative() const { return utils::RfromEulerZYZt(m_theta1,m_rho,m_theta2); }
    double getTheta1() const { return m_theta1; }
    double getRho   () const { return m_rho   ; }
    double getTheta2() const { return m_theta2; }

    // **** serialization

    void writeAdditional(cv::FileStorage &fs) override {
        fs << "im_p" + std::to_string(int(p3dImagePair_rectifiedImageLeft))  << m_imLrectified;
        fs << "im_p" + std::to_string(int(p3dImagePair_rectifiedImageRight)) << m_imRrectified;
        fs << "im_p" + std::to_string(int(p3dImagePair_disparityMap)) << m_disparityMap;
    }

    void readAdditional(const cv::FileNode &node) override {
        LOG_DBG("Maybe there is no need to store rectified images");
        // we could just regenerate them on project loading ?
        node["im_p" + std::to_string(int(p3dImagePair_rectifiedImageLeft)) ] >> m_imLrectified;
        node["im_p" + std::to_string(int(p3dImagePair_rectifiedImageRight))] >> m_imRrectified;
        node["im_p" + std::to_string(int(p3dImagePair_disparityMap))] >> m_disparityMap;
    }

    DenseMatching denseMatching{};

private:
    int _imL = -1;
    int _imR = -1;

    std::vector<Match> m_matches;

    Eigen::Matrix<double,3,3> F;

    Mat3 m_rectifyingTransformL;
    Mat3 m_rectifyingTransformR;

    cv::Mat m_imLrectified;
    cv::Mat m_imRrectified;
    cv::Mat m_disparityMap;

    double m_theta1{0.0};
    double m_rho{0.0};
    double m_theta2{0.0};

};

#endif // IMAGE_PAIR_H
