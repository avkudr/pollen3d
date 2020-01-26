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

    std::vector<cv::Point2d> getInliersLeftImageREFACTOR() { return getInliersREFACTOR(0); }
    std::vector<cv::Point2d> getInliersRightImageREFACTOR() { return getInliersREFACTOR(1); }
    std::vector<cv::Point2d> getInliersREFACTOR(int imageIdx) {return _matcher->getInliers(imageIdx);}

    void writeAdditional(cv::FileStorage &fs) override {
//        fs << "matchesTable" << _matchesTable;
//        fs << "inliersLeft" << _inliersLeft;
//        fs << "inliersRight" << _inliersRight;
    }

    void readAdditional(const cv::FileNode &node) override {
//        node["matchesTable"] >> _matchesTable;
//        node["inliersLeft"] >> _inliersLeft;
//        node["inliersRight"] >> _inliersRight;
    }


private:
    Image * _imL = nullptr;
    Image * _imR = nullptr;

    std::vector<Match> m_matches;
};


//These write and read functions must be defined for the serialization in FileStorage to work
[[maybe_unused]]
static void write(cv::FileStorage& fs, const std::string&, const ImagePair& x)
{
    fs << "{";
    const_cast<ImagePair&>(x).write(fs);
    fs << "}";
}

[[maybe_unused]]
static void read(const cv::FileNode& node, ImagePair& x, const ImagePair& default_value = ImagePair()){
    if(node.empty())
        x = default_value;
    else {
        x.read(node);
    }
}

#endif // IMAGE_PAIR_H
