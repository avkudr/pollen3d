#include "rectification.h"

using namespace p3d;

void RectificationUtil::rectify(RectificationData &data)
{
    Mat3 Tl, Tr;
    cv::Mat imLrect, imRrect;
    {
        Tr.setIdentity(3, 3);
        Tl.setIdentity(3, 3);

        cv::Mat Rl = cv::getRotationMatrix2D(
            cv::Point2d(data.imL.cols / 2.0, data.imL.rows / 2.0),
            data.angleL * 180.0 / CV_PI, 1);
        cv::Mat Rr = cv::getRotationMatrix2D(
            cv::Point2d(data.imR.cols / 2.0, data.imR.rows / 2.0),
            data.angleR * 180.0 / CV_PI, 1);

        cv::warpAffine(data.imL, imLrect, Rl, data.imL.size());
        cv::warpAffine(data.imR, imRrect, Rr, data.imR.size());
        for (int u = 0; u < 2; u++) {
            for (int v = 0; v < 3; v++) {
                Tl(u, v) = Rl.at<double>(u, v);
                Tr(u, v) = Rr.at<double>(u, v);
            }
        }
    }

    std::vector<Vec2> ptsLrect, ptsRrect;
    ptsLrect.reserve(data.ptsL.size());
    ptsRrect.reserve(data.ptsR.size());

    Vec rectErrors;
    rectErrors.setZero(data.ptsL.size(), 1);

    for (int i = 0; i < data.ptsL.size(); i++) {
        Vec3 tempPoint;
        tempPoint << data.ptsL[i][0], data.ptsL[i][1], 1;
        tempPoint = Tl * tempPoint;
        ptsLrect.emplace_back(Vec2(tempPoint(0), tempPoint(1)));

        tempPoint << data.ptsR[i][0], data.ptsR[i][1], 1;
        tempPoint = Tr * tempPoint;
        ptsRrect.emplace_back(Vec2(tempPoint(0), tempPoint(1)));

        rectErrors(i) = ptsLrect[i][1] - ptsRrect[i][1];
    }

    double meanErr = rectErrors.mean();

    cv::Mat imLrectShifted;
    cv::Mat imRrectShifted;
    int shift = std::round(std::abs(meanErr));

    Mat3 Tshift;
    Tshift.setIdentity(3, 3);
    Tshift(1, 2) = shift;

    if (meanErr > 0) {  // features of 2nd are higher than the 1st - shift
        // to the second
        imLrectShifted = imLrect;
        imRrectShifted = cv::Mat::zeros(shift, imRrect.cols, imRrect.type());
        imLrectShifted.push_back(imRrectShifted);
        imRrectShifted.push_back(imRrect);
        for (int i = 0; i < ptsRrect.size(); i++) { ptsRrect[i][1] += shift; }
        Tr = Tr * Tshift;
    } else {
        imRrectShifted = imRrect;
        imLrectShifted = cv::Mat::zeros(shift, imLrect.cols, imLrect.type());
        imRrectShifted.push_back(imLrectShifted);
        imLrectShifted.push_back(imLrect);
        for (int i = 0; i < ptsLrect.size(); i++) { ptsLrect[i][1] += shift; }
        Tl = Tl * Tshift;
    }

    for (int i = 0; i < data.ptsL.size(); i++)
        rectErrors(i) = ptsLrect[i][1] - ptsRrect[i][1];

    imLrect = imLrectShifted;
    imRrect = imRrectShifted;

    // ***** set output params

    data.Tl = Tl;
    data.Tr = Tr;
    data.imLrect = imLrect.clone();
    data.imRrect = imRrect.clone();

    data.errorMean = rectErrors.mean();
    data.errorStd =
        std::sqrt((rectErrors.array() - rectErrors.mean()).square().sum() /
                  (rectErrors.size() - 1));
}
