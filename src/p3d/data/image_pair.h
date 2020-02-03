#ifndef IMAGE_PAIR_H
#define IMAGE_PAIR_H

#include <vector>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>

#include "p3d/core/core.h"
#include "p3d/data/image.h"
#include "p3d/core/densematcher.h"
#include "p3d/core/fundmat.h"
#include "p3d/core/rectification.h"
#include "p3d/core/serialization.h"

using std::vector;
using std::string;

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
};

class ImagePair : public Serializable<ImagePair>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    ImagePair(int imL = -1, int imR = -1);
    ~ImagePair() override;

    bool isValid() { return _imL >= 0 && _imR >= 0;}
    bool hasMatches() { return getNbMatches() > 0; }
    int getNbMatches() { return int(m_matches.size()); }
    const std::vector<Match> & getMatches() const { return m_matches; }
    void getMatchesAsMap(std::map<int,int> & map) const;
    void setMatches(const std::vector<Match> & matches) { m_matches = matches; }

    inline bool operator==(const ImagePair& i) const{
        return m_matches.size() == i.m_matches.size();
    }

    Rectifier * rectifier;
    DenseMatcher * denseMatcher;

    cv::Mat getOneStereoImage(cv::Mat im1, cv::Mat im2);

    cv::Mat plotStereoRectified();

    vector<Property*> _properties;
    //void updatePsssroperties();

    int imL() { return _imL; }
    int imR() { return _imR; }
    void setLeftImage(int imL) { _imL = imL; }
    void setRightImage(int imR) { _imR = imR; }

    // ***** Epipolar geometry

    bool hasF() const;
    void setHasF(bool hasF);
    const Mat3 & getFundMat() const { return F; }
    void setFundMat(const Mat3 & f) { F = f; m_hasF = true;}
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
    void setRectifiedImageL(cv::Mat & im) { m_imLrectified = im; }
    void setRectifiedImageR(cv::Mat & im) { m_imRrectified = im; }

    Mat3 getRectifyingTransformL() const { return m_rectifyingTransformL;}
    Mat3 getRectifyingTransformR() const { return m_rectifyingTransformR;}
    void setRectifyingTransformL(const Mat3 & m) { m_rectifyingTransformL = m;}
    void setRectifyingTransformR(const Mat3 & m) { m_rectifyingTransformR = m;}


    void writeAdditional(cv::FileStorage &fs) override {
        fs << "im_p" + std::to_string(int(p3dImagePair_rectifiedImageLeft))  << m_imLrectified;
        fs << "im_p" + std::to_string(int(p3dImagePair_rectifiedImageRight)) << m_imRrectified;
    }

    void readAdditional(const cv::FileNode &node) override {
        LOG_DBG("Maybe there is no need to store rectified images");
        // we could just regenerate them on project loading ?
        node["im_p" + std::to_string(int(p3dImagePair_rectifiedImageLeft)) ] >> m_imLrectified;
        node["im_p" + std::to_string(int(p3dImagePair_rectifiedImageRight))] >> m_imRrectified;
    }

private:
    int _imL = -1;
    int _imR = -1;

    std::vector<Match> m_matches;

    Eigen::Matrix<double,3,3> F;
    bool m_hasF = false;

    Mat3 m_rectifyingTransformL;
    Mat3 m_rectifyingTransformR;

    cv::Mat m_imLrectified;
    cv::Mat m_imRrectified;
};

#endif // IMAGE_PAIR_H
