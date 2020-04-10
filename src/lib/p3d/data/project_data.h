#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "p3d/core.h"
#include "p3d/data/image.h"
#include "p3d/data/image_pair.h"
#include "p3d/data/point_cloud_container.h"
#include "p3d/logger.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dData;
enum p3dData_ {
    p3dData_projectData = 2000,
    p3dData_projectPath = 2001,
    p3dData_dummy = 2002,
    p3dData_images = 2003,
    p3dData_imagePairs = 2004,
    p3dData_measMat = 2005,
    p3dData_measMatFull = 2006,
    p3dData_pointCloudCtnr,
};

class ProjectData : public Serializable<ProjectData>{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    ProjectData();

    const std::vector<Image> &getImageList() const { return m_images; }
    void setImageList(const std::vector<Image> & imList);
    const std::vector<ImagePair> & getImagePairs() const { return m_imagesPairs; }
    void setImagePairs(const std::vector<ImagePair> &imPairs);

    void clear();
    size_t nbImages() const { return m_images.size(); }
    size_t nbImagePairs() const { return m_imagesPairs.size(); }
    bool empty() const { return nbImages() == 0; }

    std::string getProjectPath() const { return m_projectPath; }
    void setProjectPath(const std::string & path) {
        m_projectPath = path;
    }

    Image * image(const std::size_t idx) {
        if (idx >= m_images.size()) return nullptr;
        return &m_images[idx];
    }

    ImagePair * imagePair(const std::size_t idx) {
        if (idx >= m_imagesPairs.size()) return nullptr;
        return &m_imagesPairs[idx];
    }

    // ***** image pair (simplified accessors)

    Image *imagePairL(const std::size_t idx);
    Image *imagePairR(const std::size_t idx);

    void getPairwiseMatches(const std::size_t idx, std::vector<Vec2> &ptsL,
                            std::vector<Vec2> &ptsR);
    void getEpipolarErrorsResidual(const std::size_t idx, Vec & errorsSquared);
    void getEpipolarErrorsDistance(const std::size_t idx, Mat2X &distances);

    // ***** measurement matrix

    const Mat &getMeasurementMatrix() const { return m_measurementMatrix; }
    void setMeasurementMatrix(const Mat & W) { m_measurementMatrix = W; }
    const Mat & getMeasurementMatrixFull() const { return m_measurementMatrixFull; } // [3*nbImages,nbPts]
    void setMeasurementMatrixFull(const Mat &Wf)
    {
        m_measurementMatrixFull = Wf;
    }

    // ***** cameras (intrinsic, extrinsic)

    void getCamerasIntrinsics(std::vector<Vec3> *cam) const;
    void setCamerasIntrinsics(std::vector<Vec3> &cam);

    void getCamerasRotations(std::vector<Mat3> *R) const;
    void getCamerasExtrinsics(std::vector<Vec3> *Rvec,
                              std::vector<Vec2> *t) const;
    void setCamerasExtrinsics(std::vector<Vec3> &Rvec, std::vector<Vec2> &t);

    std::vector<Mat34> getCameraMatrices() const;
    Mat getCameraMatricesMat() const;

    // ***** point cloud

    PointCloudContainer &pointCloudCtnr() { return m_pointCloudCtnr; }
    const PointCloudContainer &getPointCloudCtnr() const;
    void setPointCloudCtnr(const PointCloudContainer &pointCloudCtnr);

private:
    Image *imagePairImage(const std::size_t idx, bool left)
    {
        if (idx >= m_imagesPairs.size()) return nullptr;
        int idxImage = left ? m_imagesPairs[idx].imL() : m_imagesPairs[idx].imR();
        if (idxImage >= m_images.size()) return nullptr;
        return &m_images[idxImage];
    }

    std::vector<Image> m_images{};
    std::vector<ImagePair> m_imagesPairs{};
    std::string m_projectPath{""};

    Mat m_measurementMatrix{};
    Mat m_measurementMatrixFull{};

    PointCloudContainer m_pointCloudCtnr{};
};
}  // namespace p3d
