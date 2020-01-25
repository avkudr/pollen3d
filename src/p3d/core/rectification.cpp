#include "rectification.h"
#include "p3d/console_logger.h"

Rectifier::Rectifier()
{

}

Rectifier::Rectifier(Image *im1, Image *im2, FundMat *F, std::vector<cv::Point2d> *inliers1, std::vector<cv::Point2d> *inliers2)
{
    init(im1, im2, F, inliers1, inliers2);
}

Rectifier::~Rectifier()
{

}

void Rectifier::init(Image *im1, Image *im2, FundMat *F, std::vector<cv::Point2d>* inliers1, std::vector<cv::Point2d>* inliers2) {
    _imL = im1;
    _imR = im2;
    _F = F;

    _inliers1 = inliers1;
    _inliers2 = inliers2;
}

void Rectifier::rectify()
{
    LOG_INFO("Rectification started... ");
    cv::Mat epR = _F->getEpilinesRight();
    cv::Mat epL = _F->getEpilinesLeft();

    //std::cout << _F.toMat() << "sdsdsds\n\n" << _F.toMat().t() << "\n\n";

    angleL = atan(-epL.at<double>(0)/epL.at<double>(1));
    angleR = atan(-epR.at<double>(0)/epR.at<double>(1));

    LOG_INFO( "Angles (L,R) : (%.3f,%.3f)", angleL, angleR);

    cv::Mat Rl = get2DRotationMatrixLeft();
    cv::Mat Rr = get2DRotationMatrixRight();

    cv::warpAffine(_imL->get(), imLrect, Rl, _imL->get().size());
    cv::warpAffine(_imR->get(), imRrect, Rr, _imR->get().size());

    cv::cvtColor(imLrect, imLrect, CV_BGR2GRAY);
    cv::cvtColor(imRrect, imRrect, CV_BGR2GRAY);

    cv::Mat imLrectShifted;
    cv::Mat imRrectShifted;

    std::vector<cv::Point2d> inliersLeftTransformed;
    std::vector<cv::Point2d> inliersRightTransformed;

    for(int i = 0; i < _inliers1->size(); i++) {
        cv::Mat tempPoint;
        tempPoint = (cv::Mat_<double>(3,1) << (*_inliers1)[i].x, (*_inliers1)[i].y, 1);
        tempPoint = Rl*tempPoint;
        inliersLeftTransformed.push_back(cv::Point2d(tempPoint.at<double>(0,0),tempPoint.at<double>(1,0)));

        tempPoint = (cv::Mat_<double>(3,1) << (*_inliers2)[i].x, (*_inliers2)[i].y, 1);
        tempPoint = Rr*tempPoint;
        inliersRightTransformed.push_back(cv::Point2d(tempPoint.at<double>(0,0),tempPoint.at<double>(1,0)));
    }

//    double y1mean = 0, y2mean = 0;
//    for(int i = 0; i < inliersLeftTransformed.size(); i++) {
//        y1mean = y1mean + inliersLeftTransformed[i].y;
//        y2mean = y2mean + inliersRightTransformed[i].y;
//    }
//    y1mean = y1mean/inliersLeftTransformed.size();
//    y2mean = y2mean/inliersLeftTransformed.size();

//    shift = round(abs(y2mean - y1mean));
//    if (y1mean > y2mean){ // features of 2nd are higher than the 1st - shift to the second
//        imLrectShifted = imLrect;
//        imRrectShifted = cv::Mat::zeros(shift,imRrect.cols, imRrect.type());
//        imLrectShifted.push_back(imRrectShifted);
//        imRrectShifted.push_back(imRrect);
//    }
//    else{
//        imRrectShifted = imRrect;
//        imLrectShifted = cv::Mat::zeros(shift,imLrect.cols, imLrect.type());
//        imRrectShifted.push_back(imLrectShifted);
//        imLrectShifted.push_back(imLrect);
//    }

//    for(int i = 0; i < inliersLeftTransformed.size(); i++) {
//        inliersLeftTransformed[i].y = inliersLeftTransformed[i].y + shift;
//    }

    cv::Mat rectErrors = cv::Mat::zeros(inliersLeftTransformed.size(), 1, cv::DataType<double>::type);
    for(int i = 0; i < inliersLeftTransformed.size(); i++) {
        rectErrors.at<double>(i,0) = inliersLeftTransformed[i].y - inliersRightTransformed[i].y;
    }
    cv::Scalar mean, std;
    cv::meanStdDev ( rectErrors, mean, std );

    shift = round(abs(mean[0]));
    if (mean[0] > 0){ // features of 2nd are higher than the 1st - shift to the second
        imLrectShifted = imLrect;
        imRrectShifted = cv::Mat::zeros(shift,imRrect.cols, imRrect.type());
        imLrectShifted.push_back(imRrectShifted);
        imRrectShifted.push_back(imRrect);
        for (int i = 0; i < inliersRightTransformed.size(); i++) {
            inliersRightTransformed[i].y = inliersRightTransformed[i].y + shift;
        }
    }
    else{
        imRrectShifted = imRrect;
        imLrectShifted = cv::Mat::zeros(shift,imLrect.cols, imLrect.type());
        imRrectShifted.push_back(imLrectShifted);
        imLrectShifted.push_back(imLrect);
        for (int i = 0; i < inliersLeftTransformed.size(); i++) {
            inliersLeftTransformed[i].y = inliersLeftTransformed[i].y + shift;
        }
    }

    for(int i = 0; i < inliersLeftTransformed.size(); i++) {
        rectErrors.at<double>(i,0) = inliersLeftTransformed[i].y - inliersRightTransformed[i].y;
    }
    cv::meanStdDev ( rectErrors, mean, std );

    LOG_INFO("Rectification errors:");
    //LOG_INFO(rectErrors);
    LOG_INFO("      mean: %.3f", mean.val[0]);
    LOG_INFO("       std: %.3f", std.val[0]);

    imLrect = imLrectShifted;
    imRrect = imRrectShifted;
    LOG_INFO("Done");

//    cv::imwrite("_R01.jpg", imLrect);
    //    cv::imwrite("_R02.jpg", imRrect);
}

bool Rectifier::isReady()
{
    return (! _F->isEmpty()) && (! this->_inliers1->empty()) && (! this->_inliers2->empty());
}

cv::Mat Rectifier::get2DRotationMatrixLeft()
{
    return cv::getRotationMatrix2D(cv::Point2d(_imL->get().cols/2.0,_imL->get().rows/2.0),angleL*180.0/CV_PI,1);
}

cv::Mat Rectifier::get2DRotationMatrixRight()
{
    return cv::getRotationMatrix2D(cv::Point2d(_imR->get().cols/2.0,_imR->get().rows/2.0),angleR*180.0/CV_PI,1);
}

cv::Mat Rectifier::get2DShiftMatrix()
{
    cv::Mat shiftMat = cv::Mat::zeros( 3, 3, cv::DataType<double>::type);
    shiftMat.at<double>(1,2) = shift;
    return shiftMat;

}

void Rectifier::write(cv::FileStorage &fs) const //Write serialization for this class
{
    fs << "Rectifier_imLrect" << imLrect;
    fs << "Rectifier_imRrect" << imRrect;

    fs << "Rectifier_angleL" << angleL;
    fs << "Rectifier_angleR" << angleR;
    fs << "Rectifier_shift" << shift;
}

void Rectifier::read(const cv::FileNode &node)
{
    node["Rectifier_imLrect"] >> imLrect;
    node["Rectifier_imRrect"] >> imRrect;
    node["Rectifier_angleL"] >> angleL;
    node["Rectifier_angleR"] >> angleR;
    node["Rectifier_shift"] >> shift;
}
