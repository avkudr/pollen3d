#pragma once

#include "p3d/commands.h"
#include "p3d/core.h"
#include "p3d/data/point_cloud.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dPointCloudContainer;
enum p3dPointCloudContainer_ {
    p3dPointCloudContainer_clouds = 0,
};

/**
 * @brief PointCloudContainer is a simple container with the implementation
 * similar to std::map<const char*,PointCloud>
 *
 * It is not implemented as a map because the serialization of std::map
 * is not implemented => technical debt. However, in this particular case it
 * should not be problem because (IMHO) a project with more than 10 point clouds
 * is rare
 */

class PointCloudContainer : public Serializable<PointCloudContainer>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    PointCloudContainer() {}

    bool operator==(const PointCloudContainer& i) const;

    /**
     * Returns a reference to the mapped value of the element with key
     * equivalent to key. If no such element exists, an exception of type
     * std::out_of_range is thrown.
     */
    PointCloud& at(const char* lbl);
    const PointCloud& at(const char* lbl) const;

    /**
     * Returns a reference to the value that is mapped to a key equivalent to
     * key, performing an insertion if such key does not already exist.
     */
    PointCloud& operator[](const char* lbl) noexcept;

    bool contains(const char* label) const;

    PointCloud* create(const char* lbl);
    void erase(const std::string& lbl);
    void erase(const char* lbl);
    size_t size() const { return m_pointClouds.size(); }

    std::vector<PointCloud> getPointClouds() const;
    void setPointClouds(const std::vector<PointCloud>& setPointClouds);

    std::vector<PointCloud>::iterator begin();
    std::vector<PointCloud>::iterator end();

    const std::vector<PointCloud>::const_iterator begin() const;
    const std::vector<PointCloud>::const_iterator end() const;

private:
    std::vector<PointCloud> m_pointClouds;
};

class CommandPointCloudAdd : public Command
{
public:
    CommandPointCloudAdd(PointCloudContainer* ctnr, const char* lbl,
                         const Mat3Xf& pcd, const Mat3Xf& colors = {})
        : m_lbl(lbl), m_ctnr(ctnr), m_vertices(pcd), m_colors(colors)
    {
        m_isValid = true;
        if (lbl == "") m_isValid = false;
        if (pcd.cols() == 0) m_isValid = false;
        if (ctnr == nullptr) m_isValid = false;
    }
    virtual ~CommandPointCloudAdd() {}
    void undo()
    {
        if (m_ctnr) m_ctnr->erase(m_lbl);
    }
    void execute()
    {
        m_ctnr->operator[](m_lbl).setVertices(m_vertices);
        m_ctnr->operator[](m_lbl).setColors(m_colors);
    }

protected:
    const char* m_lbl;
    PointCloudContainer* m_ctnr;
    Mat3Xf m_vertices;
    Mat3Xf m_colors;
};

class CommandPointCloudDelete : public Command
{
public:
    CommandPointCloudDelete(PointCloudContainer* ctnr, const char* lbl)
        : m_lbl(lbl), m_ctnr(ctnr)
    {
        m_isValid = true;
        if (lbl == "") m_isValid = false;
        if (ctnr == nullptr) m_isValid = false;
        if (!ctnr->contains(lbl)) m_isValid = false;
        if (m_isValid) {
            m_lbl = lbl;
            m_vertices = ctnr->at(lbl).getVertices();
            m_colors = ctnr->at(lbl).getColors();
        }
    }
    virtual ~CommandPointCloudDelete() {}
    void undo()
    {
        m_ctnr->operator[](m_lbl).setVertices(m_vertices);
        m_ctnr->operator[](m_lbl).setColors(m_colors);
    }
    void execute()
    {
        if (m_ctnr) m_ctnr->erase(m_lbl);
    }

protected:
    const char* m_lbl;
    PointCloudContainer* m_ctnr;
    Mat3Xf m_vertices;
    Mat3Xf m_colors;
};

}  // namespace p3d
