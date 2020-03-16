#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "p3d/data/image.h"
#include "p3d/data/image_pair.h"
#include "p3d/console_logger.h"
#include "p3d/core/serialization.h"
#include "p3d/core/core.h"

#ifdef WITH_OPTIM
#include <p3d/optimization.h>
#endif

using std::vector;
using std::string;

typedef int p3dData;
enum p3dData_
{
    p3dData_projectData = 2000,
    p3dData_projectPath = 2001,
    p3dData_dummy       = 2002,
    p3dData_images      = 2003,
    p3dData_imagePairs  = 2004,
    p3dData_measMat     = 2005,
    p3dData_measMatFull = 2006,
    p3dData_pts3DSparse = 2007,
    p3dData_pts3DDense  = 2008,
};

class ProjectData : public Serializable<ProjectData>{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    ProjectData();

    const std::vector<Image> & getImageList() const { return m_images; }
    void setImageList(const std::vector<Image> & imList);
    const std::vector<ImagePair> & getImagePairs() const { return m_imagesPairs; }
    void setImagePairs(const std::vector<ImagePair> & imPairs) {
        if (m_images.size() - 1 != imPairs.size()) {
            LOG_ERR("Can't load imagePairs: %i != %i", m_images.size() - 1, imPairs.size());
            return;
        }
        m_imagesPairs.clear();
        m_imagesPairs = imPairs;
    }

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

    Image * imagePairL(const std::size_t idx) { return imagePairImage(idx,true); }
    Image * imagePairR(const std::size_t idx) { return imagePairImage(idx,false); }

    void getPairwiseMatches(const std::size_t idx, std::vector<Vec2> & ptsL, std::vector<Vec2> & ptsR);
    void getEpipolarErrorsResidual(const std::size_t idx, Vec & errorsSquared);
    void getEpipolarErrorsDistance(const std::size_t idx, Mat2X &distances);

    const Mat & getMeasurementMatrix() const { return m_measurementMatrix; }
    void setMeasurementMatrix(const Mat & W) { m_measurementMatrix = W; }
    const Mat & getMeasurementMatrixFull() const { return m_measurementMatrixFull; } // [3*nbImages,nbPts]
    void setMeasurementMatrixFull(const Mat & Wf) { m_measurementMatrixFull = Wf; }

    void getCamerasIntrinsics(std::vector<Vec3> * cam) const;
    void setCamerasIntrinsics(std::vector<Vec3> & cam);
    void getCamerasExtrinsics(std::vector<Vec3> *R, std::vector<Vec2> * t) const;
    void setCamerasExtrinsics(std::vector<Vec3> &R, std::vector<Vec2> & t);

    std::vector<Mat34> getCameraMatrices();
    Mat getCameraMatricesMat();

    const Mat4X & getPts3DSparse() const { return m_pts3DSparse; }
    void setPts3DSparse(const Mat4X &pts3D) { m_pts3DSparse = pts3D; }
    const Mat4X & getPts3DDense() const { return m_pts3DDense; }
    void setPts3DDense(const Mat4X &pts3D) { m_pts3DDense = pts3D; }
private:

    Image * imagePairImage(const std::size_t idx, bool left) {
        if (idx >= m_imagesPairs.size()) return nullptr;
        int idxImage = left ? m_imagesPairs[idx].imL() : m_imagesPairs[idx].imR();
        if (idxImage >= m_images.size()) return nullptr;
        return &m_images[idxImage];
    }

    std::vector<Image> m_images{};
    std::vector<ImagePair> m_imagesPairs{};
    std::string m_projectPath = "";

    Mat m_measurementMatrix{Mat::Zero(0,0)};
    Mat m_measurementMatrixFull{Mat::Zero(0,0)};

    Mat4X m_pts3DSparse;
    Mat4X m_pts3DDense;
};
