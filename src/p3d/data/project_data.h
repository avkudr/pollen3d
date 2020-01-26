#pragma once

#include <opencv2/core/core.hpp>

#include <vector>

#include "p3d/core/imageproperties.h"
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

class ProjectData : public Serializable<ProjectData>{

public:
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

    Image * getREFACTOR(const int index);
    ImagePair * getPairREFACTOR(const int index);

    std::string getProjectPath() const { return m_projectPath; }
    void setProjectPath(const std::string & path) {
        m_projectPath = path;
    }

    void estimateMeasurementMatrixFull();
    void estimateMeasurementMatrix();
    void estimateMeasurementMatrixKLT();
    cv::Mat getMeasurementMatrix() const { return _measurementMatrix.clone();}

    cv::Mat _Pm;
    void updateCameraMatrices();
    cv::Mat getCameraMatrices(){
        return _Pm;
    }
    std::vector<cv::Mat> getCameraMatricesInArray() const {
        if (!_Pm.empty()){
            std::vector<cv::Mat> P;
            for (size_t i = 0; i < m_images.size(); i++){
                P.push_back(m_images[i].getCameraMatrix().clone());
            }
            return P;
        }
        std::vector<cv::Mat> P;
        return P;
    }

    vector<Property*> _properties;

    Image * image(const std::size_t idx) {
        if (idx < 0 && idx >= m_images.size()) return nullptr;
        return &m_images[idx];
    }

    ImagePair * imagePair(const std::size_t idx) {
        if (idx >= m_imagesPairs.size()) return nullptr;
        return &m_imagesPairs[idx];
    }

    Image * imagePairL(const std::size_t idx) { return imagePairImage(idx,true); }
    Image * imagePairR(const std::size_t idx) { return imagePairImage(idx,false); }

private:

    Image * imagePairImage(const std::size_t idx, bool left) {
        if (idx >= m_imagesPairs.size()) return nullptr;
        int idxImage = left ? m_imagesPairs[idx].imL() : m_imagesPairs[idx].imR();
        if (idxImage >= m_images.size()) return nullptr;
        return &m_images[idxImage];
    }

    std::vector<Image> m_images;
    std::vector<ImagePair> m_imagesPairs;
    std::string m_projectPath = "";


    cv::Mat _measurementMatrix;
    cv::Mat _measurementMatrixFull;
    std::vector<cv::Mat> _measMatFromDisp;

    cv::Mat _matchesTableFull;
    std::vector<cv::Point3d> _sparsePointCloud;
    cv::Mat _sparsePointCloudMat;
};
