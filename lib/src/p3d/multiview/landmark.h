#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

namespace p3d
{
enum P3D_API p3dObservation_ {
    p3dObservation_point = 0,
    p3dObservation_confidence = 1,
};

struct P3D_API Observation : public Serializable<Observation> {
    Observation() {}
    Observation(const Vec2& pt_, float confidence_ = 1.0f)
        : pt{pt_}, confidence{confidence_}
    {
    }

    static int initMeta();
    static const char* classNameStatic() { return "Observation"; }

    Vec2 pt{0, 0};
    float confidence{1.0f};
};

enum P3D_API p3dLandmark_ { p3dLandmark_X = 0, p3dLandmark_obs = 1 };

struct P3D_API Landmark : public Serializable<Landmark> {
    static int initMeta();
    static const char* classNameStatic() { return "Landmark"; }

    Vec3 X{0, 0, 0};
    std::map<int, Observation> obs;
};

struct ObservationUtil {
    static std::vector<std::map<int, Observation>> fromMeasMat(const Mat& W);
};
}  // namespace p3d
