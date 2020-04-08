#include "point_cloud_container.h"

#include <stdexcept>

namespace p3d
{
int dummyPointCloudCtnr_ = PointCloudContainer::initMeta();

int PointCloudContainer::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: PointCloudContainer" << std::endl;
        entt::meta<PointCloudContainer>()
            .type("PointCloudContainer"_hs)
            .data<&PointCloudContainer::setPointClouds,
                  &PointCloudContainer::getPointClouds>(
                P3D_ID_TYPE(p3dPointCloudContainer_clouds));

        SERIALIZED_ADD_READ_WRITE(PointCloudContainer,
                                  "PointCloudContainer"_hs);
        SERIALIZE_TYPE_VECS(PointCloudContainer,
                            "vector_PointCloudContainer"_hs);
        firstCall = false;
    }
    return 0;
}

bool PointCloudContainer::operator==(const PointCloudContainer &i) const
{
    for (auto &pcd : m_pointClouds) {
        const char *lbl = pcd.getLabel().c_str();
        if (!i.contains(lbl)) return false;

        if (!(i.at(lbl) == pcd)) return false;
    }
    return true;
}

PointCloud &PointCloudContainer::at(const char *lbl)
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return pcd;

    throw std::out_of_range("PointCloudContainer: out of range");
}

const PointCloud &PointCloudContainer::at(const char *lbl) const
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return pcd;

    throw std::out_of_range("PointCloudContainer: out of range");
}

PointCloud &PointCloudContainer::operator[](const char *lbl) noexcept
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return pcd;

    m_pointClouds.push_back(PointCloud(lbl, {}));
    return m_pointClouds.back();
}

bool PointCloudContainer::contains(const char *label) const
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == label) return true;

    return false;
}

PointCloud *PointCloudContainer::create(const char *lbl)
{
    LOG_WARN("Check for duplicates and resolve otherwise");
    m_pointClouds.push_back(PointCloud(lbl, {}));
    return &m_pointClouds.back();
}

void PointCloudContainer::erase(const std::string &lbl) { erase(lbl.c_str()); }

void PointCloudContainer::erase(const char *lbl)
{
    auto it = m_pointClouds.begin();
    while (it != m_pointClouds.end()) {
        if (lbl == it->getLabel()) {
            m_pointClouds.erase(it);
            return;
        }
        ++it;
    }
}

std::vector<PointCloud> PointCloudContainer::getPointClouds() const
{
    return m_pointClouds;
}

void PointCloudContainer::setPointClouds(
    const std::vector<PointCloud> &pointClouds)
{
    m_pointClouds = pointClouds;
}

std::vector<PointCloud>::iterator PointCloudContainer::begin()
{
    return m_pointClouds.begin();
}

std::vector<PointCloud>::iterator PointCloudContainer::end()
{
    return m_pointClouds.end();
}

const std::vector<PointCloud>::const_iterator PointCloudContainer::begin() const
{
    return m_pointClouds.begin();
}

const std::vector<PointCloud>::const_iterator PointCloudContainer::end() const
{
    return m_pointClouds.end();
}

}  // namespace p3d
