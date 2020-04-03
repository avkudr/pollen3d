#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

namespace p3d
{
P3D_EXPORTS enum p3dMatch_ {
    p3dMatch_iPtL = 100,
    p3dMatch_iPtR = 101,
    p3dMatch_distance = 102,
};

P3D_EXPORTS class Match : public Serializable<Match>
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

    inline bool operator==(const Match& a) const
    {
        return iPtL == a.iPtL && iPtR == a.iPtR;
    }
};
}  // namespace p3d

