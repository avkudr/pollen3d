#ifndef IMAGE_PAIR_H
#define IMAGE_PAIR_H

#include <vector>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core.h"
#include "p3d/data/image.h"
#include "p3d/serialization.h"
#include "p3d/stereo/dense_matching.h"
#include "p3d/stereo/fundmat.h"
#include "p3d/stereo/matching.h"

namespace p3d
{
P3D_EXPORTS enum p3dImagePair_ {
    p3dImagePair_matches = 0,
    p3dImagePair_imL = 1,
    p3dImagePair_imR = 2,
    p3dImagePair_fundMat = 3,
    p3dImagePair_hasF = 4,
    p3dImagePair_rectifyingTransformLeft,
    p3dImagePair_rectifyingTransformRight,
    p3dImagePair_rectifiedImageLeft,
    p3dImagePair_rectifiedImageRight,
    p3dImagePair_disparityMap,
    p3dImagePair_Theta1,  // theta_1
    p3dImagePair_Rho,     // rho
    p3dImagePair_Theta2,  // theta_2
    p3dImagePair_denseMatchingPars,
    p3dImagePair_matchingPars,
};

P3D_EXPORTS class ImagePair : public Serializable<ImagePair>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    ImagePair(int imL = -1, int imR = -1);
    ~ImagePair() override;

    bool operator==(const ImagePair& i) const;

    int imL() { return _imL; }
    int imR() { return _imR; }
    void setLeftImage(int imL) { _imL = imL; }
    void setRightImage(int imR) { _imR = imR; }
    bool isValid() { return _imL >= 0 && _imR >= 0; }

    // ***** Matches

    bool hasMatches() { return getNbMatches() > 0; }
    int getNbMatches() { return int(m_matches.size()); }
    const std::vector<Match> &getMatches() const { return m_matches; }
    void getMatchesAsMap(std::map<int, int> &map) const;
    void setMatches(const std::vector<Match> &matches) { m_matches = matches; }

    MatchingPars *matchingPars();
    const MatchingPars &getMatchingPars() const;
    void setMatchingPars(const MatchingPars &d);

    // ***** Epipolar geometry

    bool hasF() const;
    const Mat3 & getFundMat() const { return F; }
    void setFundMat(const Mat3 & f) { F = f; }
    void getEpilines(const std::vector<Vec2> & pts, std::vector<Vec3> & epilines, bool transpose) const;
    void getEpilinesLeft(const std::vector<Vec2> & ptsR, std::vector<Vec3> & epilinesL) const { getEpilines(ptsR, epilinesL, true);}
    void getEpilinesRight(const std::vector<Vec2> & ptsL, std::vector<Vec3> & epilinesR) const { getEpilines(ptsL, epilinesR, false); }

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
    void setDisparityMap(const cv::Mat &im) { m_disparityMap = im; }

    DenseMatchingPars *denseMatchingPars();
    const DenseMatchingPars &getDenseMatchingPars() const;
    void setDenseMatchingPars(const DenseMatchingPars &d);

    // **** rotation parameters

    void setTheta1(double a) { m_theta1 = a; }
    void setRho   (double a) { m_rho    = a; }
    void setTheta2(double a) { m_theta2 = a; }

    bool  hasRrelative() { return !utils::floatEq(m_rho,0.0); }
    const Mat3 getRrelative() const;
    double getTheta1() const { return m_theta1; }
    double getRho   () const { return m_rho   ; }
    double getTheta2() const { return m_theta2; }

    // **** serialization

    void writeAdditional(cv::FileStorage &fs) override;
    void readAdditional(const cv::FileNode &node) override;

private:
    int _imL = -1;
    int _imR = -1;

    std::vector<Match> m_matches;

    Eigen::Matrix<double, 3, 3> F;

    Mat3 m_rectifyingTransformL;
    Mat3 m_rectifyingTransformR;

    cv::Mat m_imLrectified;
    cv::Mat m_imRrectified;
    cv::Mat m_disparityMap;

    double m_theta1{0.0};
    double m_rho{0.0};
    double m_theta2{0.0};

    DenseMatchingPars m_denseMatchingPars{};
    MatchingPars m_matchingPars{};
};
}  // namespace p3d
#endif // IMAGE_PAIR_H
