#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dPointCloud;
enum P3D_API p3dPointCloud_ {
    p3dPointCloud_label = 0,
    p3dPointCloud_vertices = 1,
    p3dPointCloud_colors = 2,
    p3dPointCloud_visible = 3,
};

class P3D_API PointCloud : public Serializable<PointCloud>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();
    static const char* classNameStatic() { return "PointCloud"; }

    PointCloud() {}
    PointCloud(const std::string& label, const Mat3Xf& pcd,
               const Mat3Xf& colors = {});

    bool operator==(const PointCloud& i) const override;

    std::string getLabel() const;
    void setLabel(const std::string& label);

    const Mat3Xf& getVertices() const;
    void setVertices(const Mat3Xf& matrix);

    const Mat3Xf& getColors() const;
    void setColors(const Mat3Xf& colors);

    int nbPoints() const { return m_vertices.cols(); }

    bool isVisible() const;
    void setVisible(bool visible);

private:
    std::string m_label{""};
    Mat3Xf m_vertices{};
    Mat3Xf m_colors{};
    bool m_visible{true};
};
}  // namespace p3d
