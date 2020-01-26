#ifndef IMAGE_PAIR_H
#define IMAGE_PAIR_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/core.h"
#include "p3d/data/image.h"
#include "p3d/core/matching.h"
#include "p3d/core/densematcher.h"
#include "p3d/core/fundmat.h"
#include "p3d/core/rectification.h"
#include "p3d/core/serialization.h"

using std::vector;
using std::string;

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
};

class ImagePair : public Serializable<ImagePair>{
public:
    static int initMeta();

    ImagePair();
    ImagePair(Image * imL, Image * imR);
    ~ImagePair() override;

    bool hasMatches() { return getNbMatches() > 0; }
    int getNbMatches() { return int(m_matches.size()); }
    const std::vector<Match> & getMatches() const { return m_matches; }
    void setMatches(const std::vector<Match> & matches) { m_matches = matches; }

    inline bool operator==(const ImagePair& i) const{
        return m_matches.size() == i.m_matches.size();
    }

    Matcher* _matcher;
    FundMat * F;
    Rectifier * rectifier;
    DenseMatcher * denseMatcher;

    cv::Mat getOneStereoImage(cv::Mat im1, cv::Mat im2);
    cv::Mat plotStereoWithMatches();
    cv::Mat plotStereoWithEpilines();
    cv::Mat plotStereoRectified();

    vector<Property*> _properties;
    //void updatePsssroperties();

    Image * imL() { return _imL; }
    Image * imR() { return _imR; }
    void setLeftImage(Image * imL) { LOG_WARN("TO DO: Replace pointers with indices"); _imL = imL; }
    void setRightImage(Image * imR) { LOG_WARN("TO DO: Replace pointers with indices"); _imR = imR; }

    std::vector<cv::Point2d> getInliersLeftImageREFACTOR() { return getInliersREFACTOR(0); }
    std::vector<cv::Point2d> getInliersRightImageREFACTOR() { return getInliersREFACTOR(1); }
    std::vector<cv::Point2d> getInliersREFACTOR(int imageIdx) {return _matcher->getInliers(imageIdx);}

private:
    Image * _imL = nullptr;
    Image * _imR = nullptr;

    std::vector<Match> m_matches;
};

#endif // IMAGE_PAIR_H
