#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

namespace p3d
{
// ***** Match

using IndexPair = std::pair<uint32_t, uint32_t>;
using PairWiseMatchesMap = std::map<IndexPair, std::vector<IndexPair>>;
// Tracks = < TrackId, { <Img,Pt>,<Img,Pt>,<Img,Pt> ...} >
using Tracks = std::map<uint32_t, std::map<uint32_t, uint32_t>>;

enum P3D_API p3dMatch_ {
    p3dMatch_iPtL = 100,
    p3dMatch_iPtR = 101,
    p3dMatch_distance = 102,
};

class P3D_API Match : public Serializable<Match>
{
public:
    static int initMeta();
    static const char* classNameStatic() { return "Match"; }

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

enum P3D_API MatchingMethod_ {
    MatchingMethod_FLANNBASED = 0,
    MatchingMethod_BRUTEFORCE = 1,
    MatchingMethod_BRUTEFORCE_L1 = 2,
    MatchingMethod_BRUTEFORCE_HAMMING = 3,
    MatchingMethod_BRUTEFORCE_HAMMINGLUT = 4,
    MatchingMethod_BRUTEFORCE_SL2 = 5
};

enum P3D_API p3dMatching_ {
    p3dMatching_method = 0,
    p3dMatching_filterCoeff = 1
};

// TO DELETE
class P3D_API MatchingPars : public Serializable<MatchingPars>
{
public:
    static int initMeta();
    static const char* classNameStatic() { return "MatchingPars"; }

    int method{MatchingMethod_BRUTEFORCE_L1};
    float filterCoeff{0.3f};
};

struct P3D_API MatchingUtil {
    static void match(const cv::Mat& descL, const cv::Mat& descR,
                      const MatchingPars& pars, std::vector<Match>& matches);

    static void matchesMapsToTable(PairWiseMatchesMap&& matchesMaps, Tracks& tracks);
};
}  // namespace p3d
