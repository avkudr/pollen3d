#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/data/image.h"
#include "p3d/core/fundmat.h"
#include "p3d/core/core.h"

class Rectifier
{
public:
    Rectifier();
    Rectifier(Image* im1, Image* im2, FundMat * F, std::vector<cv::Point2d> * inliers1, std::vector<cv::Point2d> * inliers2);
    ~Rectifier();

    void init(Image* im1, Image* im2, FundMat * F, std::vector<cv::Point2d> * inliers1, std::vector<cv::Point2d> * inliers2);

    bool isRectificationDone() {return !(imLrect.empty()) && !(imRrect.empty());}

    cv::Mat imLrect;
    cv::Mat imRrect;

    double angleL = 0;
    double angleR = 0;
    int    shift  = 0;

    void rectify();
    bool isReady();

    cv::Mat get2DRotationMatrixLeft();
    cv::Mat get2DRotationMatrixRight();

    cv::Mat get2DShiftMatrix();

    cv::Mat get2DTransformationMatrixLeft()  { return get2DRotationMatrixLeft()  * get2DShiftMatrix(); }
    cv::Mat get2DTransformationMatrixRight() { return get2DRotationMatrixRight() * get2DShiftMatrix(); }

    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

private:

    Image* _imL;
    Image* _imR;
    FundMat *_F;

    std::vector<cv::Point2d> *_inliers1;
    std::vector<cv::Point2d> *_inliers2;
};

static void read(const cv::FileNode& node, Rectifier& x, const Rectifier& default_value = Rectifier()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

