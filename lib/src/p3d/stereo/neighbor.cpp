#include "neighbor.h"

using namespace p3d;

int dummyNeighbor_ = Neighbor::initMeta();

int Neighbor::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: Neighbor");

        SERIALIZED_ADD_READ_WRITE(Neighbor);

        entt::meta<Neighbor>()
            .data<&Neighbor::setFundMat, &Neighbor::getFundMat>(
                P3D_ID_TYPE(p3dNeighbor_fundMat))
            .data<&Neighbor::setTheta1, &Neighbor::getTheta1>(
                P3D_ID_TYPE(p3dNeighbor_theta1))
            .data<&Neighbor::setTheta2, &Neighbor::getTheta2>(
                P3D_ID_TYPE(p3dNeighbor_theta2))
            .data<&Neighbor::setMatches, &Neighbor::getMatches>(
                P3D_ID_TYPE(p3dNeighbor_matches))
            .data<&Neighbor::dispRange>(P3D_ID_TYPE(p3dNeighbor_dispRange))
            .data<&Neighbor::setTl, &Neighbor::Tl>(
                P3D_ID_TYPE(p3dNeighbor_rectifyingTransformLeft))
            .data<&Neighbor::setTr, &Neighbor::Tr>(
                P3D_ID_TYPE(p3dNeighbor_rectifyingTransformRight));

        firstCall = false;

        SERIALIZE_TYPE_VECS(Neighbor);
        SERIALIZE_TYPE_MAPS(Neighbor);
    }
    return 0;
}

std::vector<std::pair<uint32_t, uint32_t> > Neighbor::getMatchesAsVecOfPairs() const
{
    std::vector<std::pair<uint32_t, uint32_t> > out;
    out.reserve(m_matches.size());
    for (const auto &m : m_matches) out.emplace_back(std::make_pair(m.iPtL, m.iPtR));

    return out;
}

void Neighbor::setTheta1(double a)
{
    utils::wrapHalfPI(a);
    m_theta1 = a;
}

void Neighbor::setTheta2(double a)
{
    utils::wrapHalfPI(a);
    m_theta2 = a;
}

void Neighbor::writeAdditional(cv::FileStorage &fs)
{
    fs << "im_p" + std::to_string(int(p3dNeighbor_dispMap)) << disparity;
    fs << "im_p" + std::to_string(int(p3dNeighbor_confidenceMap)) << confidence;
}

void Neighbor::readAdditional(const cv::FileNode &node)
{
    node["im_p" + std::to_string(int(p3dNeighbor_dispMap))] >> disparity;
    node["im_p" + std::to_string(int(p3dNeighbor_confidenceMap))] >> confidence;

    LOG_ERR("Regenerate rectified images");
}
