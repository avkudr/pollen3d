#include "point_cloud_container.h"

#include <stdexcept>

namespace p3d
{
int dummyPointCloudCtnr_ = PointCloudContainer::initMeta();

int PointCloudContainer::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: PointCloudContainer");

        SERIALIZED_ADD_READ_WRITE(PointCloudContainer);

        entt::meta<PointCloudContainer>()
            .data<&PointCloudContainer::setPointClouds,
                  &PointCloudContainer::getPointClouds>(
                P3D_ID_TYPE(p3dPointCloudContainer_clouds));

        SERIALIZE_TYPE_VECS(PointCloudContainer);
        firstCall = false;
    }
    return 0;
}

bool PointCloudContainer::operator==(const PointCloudContainer &i) const
{
    for (auto &pcd : m_pointClouds) {
        const std::string &lbl = pcd.getLabel().c_str();
        if (!i.contains(lbl)) return false;

        if (!(i.at(lbl) == pcd)) return false;
    }
    return true;
}

PointCloud &PointCloudContainer::at(const std::string &lbl)
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return pcd;

    throw std::out_of_range("PointCloudContainer: out of range");
}

const PointCloud &PointCloudContainer::at(const std::string &lbl) const
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return pcd;

    throw std::out_of_range("PointCloudContainer: out of range");
}

PointCloud &PointCloudContainer::operator[](const std::string &lbl) noexcept
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return pcd;

    m_pointClouds.push_back(PointCloud(lbl, {}));
    return m_pointClouds.back();
}

bool PointCloudContainer::contains(const std::string &lbl) const
{
    for (auto &pcd : m_pointClouds)
        if (pcd.getLabel() == lbl) return true;

    return false;
}

PointCloud *PointCloudContainer::create(const std::string &lbl)
{
    LOG_WARN("Check for duplicates and resolve otherwise");
    m_pointClouds.push_back(PointCloud(lbl, {}));
    return &m_pointClouds.back();
}

void PointCloudContainer::erase(const std::string &lbl)
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

size_t PointCloudContainer::size() const { return m_pointClouds.size(); }

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

std::vector<std::string> PointCloudContainer::getAllLabels() const
{
    std::vector<std::string> out;
    out.reserve(m_pointClouds.size());
    auto it = m_pointClouds.begin();
    while (it != m_pointClouds.end()) {
        out.push_back(it->getLabel());
        ++it;
    }
    return out;
}

}  // namespace p3d
