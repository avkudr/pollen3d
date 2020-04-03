/*!
 *  \brief     Class realisation of Fundamental Matrix algorithms
 *  \details   This class is used to demonstrate a number of section commands.
 *  \author    Andrey Kudryavtsev
 *  \version   0.1
 *  \date      15/05/2016
 *  \pre
 *  \bug
 */

#pragma once

#include <opencv2/core.hpp>
#include "p3d/core/core.h"
#include "p3d/core/utils.h"

struct fundmat{
    static Mat3 findAffineGS(const std::vector<Vec2> & ptsL,const std::vector<Vec2> & ptsR);
    static Mat3 findAffineCeres(const std::vector<Vec2> & ptsL, const std::vector<Vec2> & ptsR);

    template<typename T>
    static void epiporalDistances(const T a, const T b, const T c, const T d, const T e,
                               const T qx, const T qy, const T qTx, const T qTy,
                               T * errorL, T * errorR);
    static Vec2 epiporalDistancesF(const Mat3 & F, const Vec2 & qL, const Vec2 & qR);

    // Mat3 affineFromP(const Mat34 & Pl, const Mat34 & Pr);
    static std::pair<double,double> slopAngles(const Mat3 & F);
};

void normalizePoints(std::vector<cv::Point2d> & pts, cv::Mat & transformMatrix);

//cv::Mat F = (cv::Mat_<double>(3,3) << 0,0,a,0,0,b,c,d,1);
//if ( (!T1.empty()) && (!T2.empty())){
//    F = T2.t() * F * T1;
//}
//F = F / F.at<double>(2,2);

