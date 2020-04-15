#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

namespace p3d
{
// ***** Match

enum P3D_EXPORTS p3dMatch_ {
    p3dMatch_iPtL = 100,
    p3dMatch_iPtR = 101,
    p3dMatch_distance = 102,
};

class P3D_EXPORTS Match : public Serializable<Match>
{
public:
    static int initMeta();

    Match() {}
    Match(int idxPtImageL, int idxPtImageR, float dist = 0.0f)
        : iPtL(idxPtImageL), iPtR(idxPtImageR), distance(dist)
    {
    }
    ~Match() {}

    int iPtL{0};
    int iPtR{0};
    float distance = 0;

    bool operator==(const Match& a) const override
    {
        return iPtL == a.iPtL && iPtR == a.iPtR;
    }
};

enum P3D_EXPORTS p3dMatching_ {
    p3dMatching_method = 0,
    p3dMatching_filterCoeff = 1
};

enum P3D_EXPORTS MatchingMethod_ {
    MatchingMethod_FLANNBASED = 0,
    MatchingMethod_BRUTEFORCE = 1,
    MatchingMethod_BRUTEFORCE_L1 = 2,
    MatchingMethod_BRUTEFORCE_HAMMING = 3,
    MatchingMethod_BRUTEFORCE_HAMMINGLUT = 4,
    MatchingMethod_BRUTEFORCE_SL2 = 5
};

class P3D_EXPORTS MatchingPars : public Serializable<MatchingPars>
{
public:
    static int initMeta();

    int method{MatchingMethod_BRUTEFORCE_L1};
    float filterCoeff{0.3f};
};

P3D_EXPORTS struct MatchingUtil {
    static void match(const cv::Mat& descL, const cv::Mat& descR,
                      const MatchingPars& pars, std::vector<Match>& matches);
};
}  // namespace p3d
