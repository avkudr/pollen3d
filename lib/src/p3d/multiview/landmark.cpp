#include "landmark.h"

using namespace p3d;

int dummyObservation_ = Observation::initMeta();

int p3d::Observation::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: %s", classNameStatic());

        SERIALIZED_ADD_READ_WRITE(Observation);

        entt::meta<Observation>()
            .data<&Observation::pt>(P3D_ID_TYPE(p3dObservation_point))
            .data<&Observation::confidence>(P3D_ID_TYPE(p3dObservation_confidence));

        SERIALIZE_TYPE_VECS(Observation);
        SERIALIZE_TYPE_MAPS(Observation);

        firstCall = false;
    }
    return 0;
}

int dummyLandmark_ = Landmark::initMeta();

int p3d::Landmark::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: %s", classNameStatic());

        SERIALIZED_ADD_READ_WRITE(Landmark);

        entt::meta<Landmark>()
            .data<&Landmark::X>(P3D_ID_TYPE(p3dLandmark_X))
            .data<&Landmark::obs>(P3D_ID_TYPE(p3dLandmark_obs));

        SERIALIZE_TYPE_VECS(Landmark);
        SERIALIZE_TYPE_MAPS(Landmark);

        firstCall = false;
    }
    return 0;
}

std::vector<std::map<int, Observation>> p3d::ObservationUtil::fromMeasMat(const Mat &W)
{
    std::vector<std::map<int, Observation>> matches;
    int nbViews = W.rows() / 3;
    int nbPts = W.cols();
    for (int p = 0; p < nbPts; ++p) {
        std::map<int, Observation> m;
        for (int c = 0; c < nbViews; ++c) {
            if (W(3 * c + 2, p) == 1.0)
                m.insert({c, Observation(W.block(3 * c, p, 2, 1))});
        }
        matches.push_back(m);
    }
    return matches;
}

void LandmarksUtil::toMat4X(const std::vector<Landmark> &l, Mat4X &X)
{
    X.setOnes(4, l.size());
    for (int i = 0; i < l.size(); ++i) { X.col(i).topRows(3) = l[i].X; }
}

void LandmarksUtil::toMat3X(const std::vector<Landmark> &l, Mat3X &X)
{
    X.setZero(3, l.size());
    for (int i = 0; i < l.size(); ++i) { X.col(i) = l[i].X; }
}

void LandmarksUtil::from3DPtsMeasMat(const Mat3X &inX, const Mat &inW,
                                     std::vector<Landmark> &l)
{
    p3d_Assert(inX.cols() == inW.cols());

    std::vector<std::map<int, Observation>> matches = ObservationUtil::fromMeasMat(inW);

    l.clear();
    l.reserve(inX.cols());
    for (int i = 0; i < inX.cols(); ++i) {
        l.emplace_back(Landmark(matches[i], inX.col(i).topRows(3)));
    }
}

double LandmarksUtil::reprojError(const Landmark &l, const std::vector<Mat34> &P)
{
    const int nbCams = l.obs.size();
    if (nbCams == 0) return std::numeric_limits<float>::max();

    double meanErr = 0;
    for (const auto &[c, obs] : l.obs) {
        p3d_DbgAssert(c < P.size());

        const auto &Pc = P.at(c);
        const double qwinv = 1.0 / (Pc.block<1, 3>(2, 0) * l.X + P[c](2, 3));

        double qx = qwinv * (Pc.block<1, 3>(0, 0) * l.X + Pc(0, 3));
        double qy = qwinv * (Pc.block<1, 3>(1, 0) * l.X + Pc(1, 3));

        meanErr += Vec2(obs.pt(0) - qx, obs.pt(1) - qy).norm();
    }

    meanErr /= double(nbCams);
    return meanErr;
}

Vec LandmarksUtil::reprojErrors(const std::vector<Landmark> &landmarks,
                                const std::vector<Mat34> &P)
{
    Vec errs;
    errs.setOnes(landmarks.size(), 1);

    for (int i = 0; i < landmarks.size(); ++i) {
        const auto &l = landmarks[i];
        errs[i] = reprojError(l, P);
    }
    return errs;
}

bool LandmarksUtil::equalsApprox(const std::vector<Landmark> &lhs,
                                 const std::vector<Landmark> &rhs)
{
    if (lhs.size() != rhs.size()) return false;
    for (auto i = 0; i < lhs.size(); ++i) {
        if (!lhs[i].X.isApprox(rhs[i].X)) return false;
    }
    return true;
}
