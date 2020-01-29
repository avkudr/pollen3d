#ifndef IMAGE_PAIR_H
#define IMAGE_PAIR_H

#include <vector>

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
    p3dImagePair_imL = 1,
    p3dImagePair_imR = 2,
    p3dImagePair_fundMat = 3,
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
    void setMatches(const std::vector<Match> & matches) { m_matches = matches; }

    inline bool operator==(const ImagePair& i) const{
        return m_matches.size() == i.m_matches.size();
    }

    Rectifier * rectifier;
    DenseMatcher * denseMatcher;

    cv::Mat getOneStereoImage(cv::Mat im1, cv::Mat im2);

    cv::Mat plotStereoWithEpilines();
    cv::Mat plotStereoRectified();

    vector<Property*> _properties;
    //void updatePsssroperties();

    int imL() { return _imL; }
    int imR() { return _imR; }
    void setLeftImage(int imL) { _imL = imL; }
    void setRightImage(int imR) { _imR = imR; }

    const Eigen::Matrix<double,3,3> & getFundMat() const { return F; }
    void setFundMat(const Eigen::Matrix<double,3,3> & f) { F = f; }

    std::vector<cv::Point2d> getInliersLeftImageREFACTOR() { return getInliersREFACTOR(0); }
    std::vector<cv::Point2d> getInliersRightImageREFACTOR() { return getInliersREFACTOR(1); }
    std::vector<cv::Point2d> getInliersREFACTOR(int imageIdx) {return std::vector<cv::Point2d>();/*_matcher->getInliers(imageIdx);*/}

private:
    int _imL = -1;
    int _imR = -1;

    std::vector<Match> m_matches;

    Eigen::Matrix<double,3,3> F;
};

#endif // IMAGE_PAIR_H
