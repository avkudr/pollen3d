#include "point_cloud.h"

#include "p3d/utils.h"

using namespace p3d;

int dummyPointCloud_ = PointCloud::initMeta();

int PointCloud::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: PointCloud");
        entt::meta<PointCloud>()
            .alias("PointCloud"_hs)
            .data<&PointCloud::setLabel, &PointCloud::getLabel>(
                P3D_ID_TYPE(p3dPointCloud_label))
            .data<&PointCloud::setVertices, &PointCloud::getVertices>(
                P3D_ID_TYPE(p3dPointCloud_vertices))
            .data<&PointCloud::setColors, &PointCloud::getColors>(
                P3D_ID_TYPE(p3dPointCloud_colors))
            .data<&PointCloud::setVisible, &PointCloud::isVisible>(
                P3D_ID_TYPE(p3dPointCloud_visible));

        SERIALIZED_ADD_READ_WRITE(PointCloud, "PointCloud"_hs);
        SERIALIZE_TYPE_VECS(PointCloud, "vector_PointCloud"_hs);
        firstCall = false;
    }
    return 0;
}

p3d::PointCloud::PointCloud(const std::string &label, const p3d::Mat3Xf &pcd,
                            const Mat3Xf &colors)
    : Serializable(), m_label(label), m_vertices(pcd), m_colors(colors)
{
}

bool PointCloud::operator==(const PointCloud &i) const
{
    if (m_label != i.getLabel()) return false;
    if (m_vertices.cols() != i.getVertices().cols()) return false;
    if (m_colors.cols() != i.getColors().cols()) return false;

    if (!m_vertices.isApprox(i.getVertices())) return false;
    if (!m_colors.isApprox(i.getColors())) return false;
    return true;
}

std::string PointCloud::getLabel() const { return m_label; }

void PointCloud::setLabel(const std::string &label) { m_label = label; }

const Mat3Xf &PointCloud::getVertices() const { return m_vertices; }

void PointCloud::setVertices(const Mat3Xf &matrix) { m_vertices = matrix; }

const Mat3Xf &PointCloud::getColors() const { return m_colors; }

void PointCloud::setColors(const Mat3Xf &colors) { m_colors = colors; }

bool PointCloud::isVisible() const { return m_visible; }

void PointCloud::setVisible(bool visible)
{
    // sqdqssdqsd
    m_visible = visible;
}
