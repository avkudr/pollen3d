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
    ProjectData();
    void setImageList(std::vector<Image> & imList){
        if (imList.empty()) return;
        _images = imList;
        _imagesPairs.clear();
        for (auto i = 0; i < _images.size()-1; ++i) {
            _imagesPairs.emplace_back(ImagePair(&_images[i],&_images[i+1]));
        }
    }
    void clear();
    size_t nbImages() const { return _images.size(); }
    size_t nbImagePairs() const { return _imagesPairs.size(); }

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
            for (size_t i = 0; i < _images.size(); i++){
                P.push_back(_images[i].getCameraMatrix().clone());
            }
            return P;
        }
        std::vector<cv::Mat> P;
        return P;
    }

    vector<Property*> _properties;

    Image * image(const std::size_t idx) {
        if (idx < 0 && idx >= _images.size()) return nullptr;
        return &_images[idx];
    }
    Image * operator[](const std::size_t idx) {
        return image(idx);
    }

    ImagePair * imagePair(const std::size_t idx) {
        if (idx < 0 && idx >= _imagesPairs.size()) return nullptr;
        return &_imagesPairs[idx];
    }


    void writeAdditional(cv::FileStorage &fs) override {
//        if (nbImages() == 0) return;
//        fs << "imageNumber" << static_cast<int>(nbImages());
//        for (size_t i = 0; i < nbImages(); i++){
//            fs << std::string("Image" + std::to_string(i));
//            fs << "{";
//            _images[i].write(fs);
//            fs << "}";
//        }

        for (auto i = 0; i < int(_images.size()) - 1; i++){
            std::ostringstream ostr;
            ostr << "ImagePair" << i;
            fs << ostr.str();
            fs << "{";
            _imagesPairs[i].write(fs);
            fs << "}";
        }

    }

    void readAdditional(const cv::FileNode &node) override {
//        int imNb = static_cast<int>(node["imageNumber"]);
//        if (imNb == 0) {
//            LOG_INFO("Project contains no images");
//            return;
//        }

//        std::vector<Image> imgs;
//        imgs.reserve(static_cast<size_t>(imNb));
//        for (auto i = 0; i < imNb; i++){
//            std::ostringstream ostr;
//            ostr << "Image" << i;
//            Image im;
//            node[ostr.str()] >> im;
//            imgs.push_back(im);
//            if (!im.cvMat().data) {
//                LOG_ERR("One of images is missing, can't continue");
//                return;
//            }
//        }

        _imagesPairs.clear();
        for (auto i = 0; i < int(_images.size())-1; ++i) {
            _imagesPairs.emplace_back(ImagePair(&_images[i],&_images[i+1]));
        }

        for (auto i = 0; i < int(_images.size()) - 1; i++){
            std::ostringstream ostr;
            ostr << "ImagePair" << int(i);
            ImagePair imPair(&_images[i],&_images[i+1]);
            node[ostr.str()] >> imPair;
            if (i < _imagesPairs.size())
                _imagesPairs[i] = imPair;
        }
    }

    bool set_isDummy = true;

private:
    std::vector<Image> _images;
    std::vector<ImagePair> _imagesPairs;

    std::string m_projectPath = "";

    cv::Mat _measurementMatrix;
    cv::Mat _measurementMatrixFull;
    std::vector<cv::Mat> _measMatFromDisp;

    cv::Mat _matchesTableFull;
    std::vector<cv::Point3d> _sparsePointCloud;
    cv::Mat _sparsePointCloudMat;
};
