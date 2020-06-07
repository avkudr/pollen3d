#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "p3d/core.h"
#include "p3d/data/image.h"
#include "p3d/data/image_pair.h"
#include "p3d/data/point_cloud_container.h"
#include "p3d/data/project_settings.h"
#include "p3d/logger.h"
#include "p3d/multiview/autocalib.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dProject;
enum P3D_API p3dProject_ {
    p3dProject_projectSettings,
    p3dProject_projectPath,
    p3dProject_images,
    p3dProject_imagePairs,
    p3dProject_measMat,
    p3dProject_pointCloudCtnr,
    p3dProject_autocalibPars
};

class P3D_API Project : public Serializable<Project>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();
    static const char *classNameStatic() { return "Project"; }

    Project();

    const std::vector<Image> &getImageList() const { return m_images; }
    void setImageList(const std::vector<Image> &imList);

    std::map<int, std::map<int, Neighbor>> &imagePairs() { return m_pairs; }
    const std::map<int, std::map<int, Neighbor>> &getImagePairs() const
    {
        return m_pairs;
    }
    void setImagePairs(const std::map<int, std::map<int, Neighbor>> &imPairs)
    {
        m_pairs = imPairs;
    }

    bool hasPair(const int imL, const int imR)
    {
        if (m_pairs.count(imL) == 0) return false;
        return m_pairs.at(imL).count(imR) > 0;
    }

    void clear();
    size_t nbImages() const { return m_images.size(); }
    size_t nbImagePairs() const
    {
        int n = 0;
        for (const auto &kv : m_pairs) { n += kv.second.size(); }
        return n;
    }
    bool empty() const { return nbImages() == 0; }

    std::string getProjectPath() const { return m_projectPath; }
    std::string getName() const { return utils::baseNameFromPath(m_projectPath); }

    void setProjectPath(const std::string &path)
    {
        m_projectPath = path;
    }

    Image *image(const std::size_t idx)
    {
        if (idx >= m_images.size()) return nullptr;
        return &m_images[idx];
    }

    const Image &getImage(const std::size_t idx) const { return m_images[idx]; }

    ImagePair *imagePair(const std::size_t idx)
    {
        if (idx >= m_imagesPairs.size()) return nullptr;
        return &m_imagesPairs[idx];
    }

    const ImagePair &getImagePair(const std::size_t idx) const { return m_imagesPairs[idx]; }

    // ***** image pair (simplified accessors)

    Image *imagePairL(const std::size_t idx);
    Image *imagePairR(const std::size_t idx);

    void getPairwiseMatches(const std::size_t imL, const std::size_t imR,
                            std::vector<Vec2> &ptsL, std::vector<Vec2> &ptsR);
    void getEpipolarErrorsResidual(const std::size_t imL, const std::size_t imR,
                                   Vec &errorsSquared);
    void getEpipolarErrorsDistance(const std::size_t imL, const std::size_t imR,
                                   Mat2X &distances);

    // ***** measurement matrix

    const Mat &getMeasurementMatrix() const
    {
        return m_measurementMatrix;
    }  // [3*nbImages,nbPts]
    void setMeasurementMatrix(const Mat &Wf) { m_measurementMatrix = Wf; }

    // ***** cameras (intrinsic, extrinsic)

    void getCamerasIntrinsics(std::vector<Vec3> *cam) const;
    void setCamerasIntrinsics(std::vector<Vec3> &cam);

    void getCamerasExtrinsics(std::vector<Mat3> *R, std::vector<Vec2> *t) const;
    void getCamerasExtrinsics(std::vector<Vec3> *Rvec, std::vector<Vec2> *t) const;
    void setCamerasExtrinsics(std::vector<Vec3> &Rvec, std::vector<Vec2> &t);

    void getCamerasRotations(std::vector<Mat3> *R) const;  // delete 07/06/2020
    std::vector<Mat3> getCamerasRotations() const;

    std::vector<Vec3> getCameraRotationsAbsolute() const;
    void setCameraRotationsAbsolute(const std::vector<Vec3> &abs);

    std::vector<Vec3> getCameraRotationsRelative() const;
    MatX3 getCameraRotationsRelativeMat() const;
    void setCameraRotationsRelative(const MatX3 & zyzt);

    std::vector<Vec2> getCameraTranslations() const;
    void setCameraTranslations(const std::vector<Vec2> & t);

    std::vector<Mat34> getCameraMatrices() const;
    Mat getCameraMatricesMat() const;

    // ***** point cloud

    PointCloudContainer &pointCloudCtnr() { return m_pointCloudCtnr; }
    const PointCloudContainer &getPointCloudCtnr() const;
    void setPointCloudCtnr(const PointCloudContainer &pointCloudCtnr);

    ProjectSettings *settings() { return &m_settings; }
    const ProjectSettings &getSettings() const;
    void setSettings(const ProjectSettings &settings);

    const AutocalibPars &getAutocalibPars() const;
    void setAutocalibPars(const AutocalibPars &autocalibPars);

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

    AutocalibPars m_autocalibPars{};
    PointCloudContainer m_pointCloudCtnr{};
    ProjectSettings m_settings;

    std::map<int, std::map<int, Neighbor>> m_pairs;
};
}  // namespace p3d
