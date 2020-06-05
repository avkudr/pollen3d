#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>

#include "p3d/core.h"
#include "p3d/serialization.h"
#include "p3d/stereo/matching.h"
#include "p3d/utils.h"

#include <map>

namespace p3d
{
enum P3D_API p3dNeighbor_ {
    p3dNeighbor_matches = 0,
    p3dNeighbor_dispMap,
    p3dNeighbor_confidenceMap,
    p3dNeighbor_dispRange,
    p3dNeighbor_rectifyingTransformLeft,
    p3dNeighbor_rectifyingTransformRight,
    p3dNeighbor_rectifiedImageLeft,
    p3dNeighbor_rectifiedImageRight,
    p3dNeighbor_theta1,
    p3dNeighbor_theta2,
    p3dNeighbor_idImL,
    p3dNeighbor_idImR,
    p3dNeighbor_fundMat
};

class P3D_API Neighbor : public Serializable<Neighbor>
{
public:
    cv::Size imLsize;
    cv::Size imRsize;

    Neighbor(int imL = -1, int imR = -1) : m_imL{imL}, m_imR{imR} {}

    static int initMeta();
    static const char *classNameStatic() { return "Neighbor"; }

    cv::Mat imLrect;
    cv::Mat imRrect;

    Vec2f dispRange{0, 0};
    cv::Mat disparity;
    cv::Mat confidence;

    // ***** Matches

    bool hasMatches() const { return getNbMatches() > 0; }
    int nbMatches() const { return int(m_matches.size()); }
    int getNbMatches() const { return nbMatches(); }  // refactor delete
    const std::vector<Match> &getMatches() const { return m_matches; }
    void getMatchesAsMap(std::map<int, int> &map) const;
    void setMatches(const std::vector<Match> &matches) { m_matches = matches; }
    void deleteMatches() { m_matches.clear(); }

    inline bool isValid() const { return !disparity.empty(); }
    inline bool isInversed() const { return m_isInversed; }

    int imL() { return m_imL; }
    int imR() { return m_imR; }
    void setLeftImage(int imL) { m_imL = imL; }
    void setRightImage(int imR) { m_imR = imR; }
    bool isValid() { return m_imL >= 0 && m_imR >= 0 && m_imL != m_imR; }

    bool hasF() const { return !m_F.isApprox(Mat3::Zero()); }
    const Mat3 &getFundMat() const { return m_F; }
    void setFundMat(const Mat3 &f) { m_F = f; }

    double getTheta1() const { return m_theta1; }
    double getTheta2() const { return m_theta2; }
    void setTheta1(double a);
    void setTheta2(double a);

    const Mat3f &Tl() const { return m_Tl; }
    const Mat3f &Tr() const { return m_Tr; }
    const Mat3f &Trinv() const { return m_Trinv; }

    void setTl(const Mat3f &T) { m_Tl = T; }
    void setTr(const Mat3f &T)
    {
        m_Tr = T;
        m_Trinv = utils::inverseTransform(m_Tr);
    }

    Neighbor inverse()
    {
        Neighbor ninv(m_imR, m_imL);
        ninv.imLrect = imRrect.clone();
        ninv.imRrect = imLrect.clone();
        ninv.setTr(Tl());
        ninv.setTl(Tr());
        ninv.disparity = -1.0f * disparity;
        ninv.confidence = confidence.clone();
        ninv.valid = valid;
        ninv.m_isInversed = true;
        ninv.dispRange = Vec2f{0, 0};
        return ninv;
    }

    void writeAdditional(cv::FileStorage &fs) override;
    void readAdditional(const cv::FileNode &node) override;

private:
    bool m_isInversed{false};
    bool valid{true};

    double m_theta1{0.0};
    double m_theta2{0.0};

    Mat3 m_F{Mat3::Zero()};

    int m_imL{-1};
    int m_imR{-1};

    Mat3f m_Tl{Mat3f::Identity()};
    Mat3f m_Tr{Mat3f::Identity()};
    Mat3f m_Trinv{Mat3f::Identity()};

    std::vector<Match> m_matches;
};

}  // namespace p3d
