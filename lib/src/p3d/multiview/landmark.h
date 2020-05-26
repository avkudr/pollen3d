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

    Landmark() {}
    Landmark(const std::map<int, Observation>& obs_, const Vec3& X_ = Vec3(0, 0, 0))
        : obs{obs_}, X{X_}
    {
    }

    Vec3 X{0.0, 0.0, 0.0};
    std::map<int, Observation> obs;

    bool isTriangulated() const
    {
        if (X(0) != 0.0) return true;
        if (X(1) != 0.0) return true;
        if (X(2) != 0.0) return true;
        return false;
    }
};

struct ObservationUtil {
    static std::vector<std::map<int, Observation>> fromMeasMat(const Mat& W);
};

struct LandmarksUtil {
    static void toMat4X(const std::vector<Landmark>& l, Mat4X& X);
    static void toMat3X(const std::vector<Landmark>& l, Mat3X& X);
    static void from3DPtsMeasMat(const Mat3X& inX, const Mat& inW,
                                 std::vector<Landmark>& l);

    // P - a vector of camera matrices, some of them might not be used
    // by the given landmark;
    static float reprojError(const Landmark& l, const std::vector<Mat34>& P);
    static Vecf reprojErrors(const std::vector<Landmark>& l, const std::vector<Mat34>& P);
    static bool equalsApprox(const std::vector<Landmark>& lhs,
                             const std::vector<Landmark>& rhs);
};
}  // namespace p3d
