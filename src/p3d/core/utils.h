#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core.hpp>

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <fstream>

#include "p3d/core/core.h"

namespace utils{

void saveFileToMatlab(std::string fileName, cv::Mat a, std::string varName);

template<typename T>
static T rad2deg(const T& angRad){
    return angRad * 180.0 / M_PI;
}

template<typename T>
static T deg2rad(const T& angDeg){
    return angDeg * M_PI / 180.0;
}

template<typename T>
static inline bool floatEq(const T& a, const T& b, double eps = 1e-5) {
    return std::abs(a - b) < eps;
}

void makeNonHomogenious(Mat & m);
void copyMatElementsToVector(const Mat &mat, const std::vector<std::pair<int, int>> &idx, std::vector<double> &vec);
void copyVectorToMatElements(const std::vector<double> & vec, const std::vector<std::pair<int, int>> &idx, Mat &mat);

std::string baseNameFromPath(const std::string & path);
bool endsWith(std::string const & value, std::string const & ending);
std::vector<std::string> split(const std::string& str, const std::string& delim);

bool equalsCvMat(const cv::Mat & lhs, const cv::Mat & rhs);

enum{
    CONCAT_HORIZONTAL,
    CONCAT_VERTICAL,
};

cv::Mat concatenateCvMat(const std::vector<cv::Mat> & matArray, int method = CONCAT_VERTICAL);

template<typename T, int SizeX, int SizeY>
Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> concatenateMat(const std::vector<Eigen::Matrix<T,SizeX,SizeY>> & matArray, int method = CONCAT_VERTICAL)
{
    Mat outMat;
    if (method == CONCAT_VERTICAL){
        outMat.setZero(SizeX * matArray.size(), SizeY);
        int row = 0;
        for(auto& m:matArray) {
            outMat.block(row,0,SizeX,SizeY) = m;
            row += SizeX;
        }
    }else{
        outMat.setZero(SizeX, SizeY * matArray.size());
        int col = 0;
        for(auto& m:matArray) {
            outMat.block(col,0,SizeX,SizeY) = m;
            col += SizeY;
        }
    }
    return outMat;
}

int nbAvailableThreads();

std::pair<Vec2,Vec2> lineIntersectBox(const Vec3& line, double w, double h/*, double x = 0.0, double y = 0.0*/);

void matchesMapsToTable(std::vector<std::map<int, int> > matchesMaps, Mati &table);

// **** Rotations

template<typename T>
Eigen::Matrix<T, 3, 3> RfromEulerZYZ(const T& t1, const T& rho, const T& t2){
    auto c0 = std::cos(t1);
    auto c1 = std::cos(rho);
    auto c2 = std::cos(t2);
    auto s0 = std::sin(t1);
    auto s1 = std::sin(rho);
    auto s2 = std::sin(t2);

    Eigen::Matrix<T, 3, 3> R;
    R(0,0) = c0*c1*c2 - s0*s2;
    R(0,1) = -c0*c1*s2 - s0*c2;
    R(0,2) = c0*s1;
    R(1,0) = s0*c1*c2+c0*s2 ;
    R(1,1) = -s0*c1*s2 + c0*c2 ;
    R(1,2) = s0*s1;
    R(2,0) = -s1*c2;
    R(2,1) = s1*s2;
    R(2,2) = c1;
    return R;
}

// [1] page(42) eq(2.17)
template<typename T>
Eigen::Matrix<T, 3, 3> RfromEulerZYZt(const T& thetaT, const T& rho, const T& theta){
    return RfromEulerZYZ(thetaT,rho,-theta);
}

void EulerZYZfromR(const Mat3 &R, double &t1, double &rho, double &t2);

// **** Math
Mat3 skewSym(const Vec3 & a);

}

#endif // UTILS_H
