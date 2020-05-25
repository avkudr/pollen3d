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
