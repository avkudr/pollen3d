#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "p3d/core.h"

namespace p3d
{
namespace utils
{

class HideCout{
public:
    HideCout(){
        out.clear();
        coutbuf = std::cout.rdbuf(); //save old buf
        cerrbuf = std::cerr.rdbuf(); //save old buf
        clogbuf = std::clog.rdbuf(); //save old buf
        std::cout.rdbuf(out.rdbuf()); //redirect std::cout
        std::cerr.rdbuf(out.rdbuf()); //redirect std::cout
        std::clog.rdbuf(out.rdbuf()); //redirect std::cout

    }

    ~HideCout(){
        out.clear();
        std::cout.rdbuf(coutbuf); //redirect std::cout back
        std::cerr.rdbuf(cerrbuf); //redirect std::cout back
        std::clog.rdbuf(clogbuf); //redirect std::cout back
    }

private:
    std::ofstream out;
    std::streambuf * coutbuf;
    std::streambuf * cerrbuf;
    std::streambuf * clogbuf;
};

template <typename T>
static T min(T a, T b)
{
    return a < b ? a : b;
}

template <typename T>
static T max(T a, T b)
{
    return a > b ? a : b;
}

void saveFileToMatlab(std::string fileName, cv::Mat a, std::string varName);

Vec reprojectionError(const Mat & W, const Mat & P, const Mat4X & X, std::vector<int> selCams = {});

template <typename T>
static T rad2deg(const T &angRad)
{
    return angRad * 180.0 / M_PI;
}

template <typename T>
static T deg2rad(const T &angDeg)
{
    return angDeg * M_PI / 180.0;
}

template <typename T>
static inline bool floatEq(const T &a, const T &b, double eps = 1e-5)
{
    return std::abs(a - b) < eps;
}

void makeNonHomogenious(Mat &m);
void copyMatElementsToVector(const Mat &mat, const std::vector<std::pair<int, int>> &idx, std::vector<double> &vec);
void copyVectorToMatElements(const std::vector<double> &vec, const std::vector<std::pair<int, int>> &idx, Mat &mat);

std::string baseNameFromPath(const std::string &path);
bool endsWith(std::string const &value, std::string const &ending);
std::vector<std::string> split(const std::string &str, const std::string &delim);

bool equalsCvMat(const cv::Mat &lhs, const cv::Mat &rhs);

enum {
    CONCAT_HORIZONTAL,
    CONCAT_VERTICAL,
};

cv::Mat concatenateCvMat(const std::vector<cv::Mat> &matArray, int method = CONCAT_VERTICAL);

void convert(const std::vector<double> &src, Vec &dst);
void convert(const std::vector<Vec3f> &src, Mat3Xf &dst);
void convert(const std::vector<Vec3> &src, Mat3Xf &dst);
void convert(const std::vector<Vec3> &src, Mat4X &dst);
void convert(const std::vector<Vec4> &src, Mat4X &dst);
void convert(const Mat2X &src, std::vector<Vec2> &dst);

std::string to_string(const std::vector<int> &v);

template <typename T, int SizeX, int SizeY>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> concatenateMat(const std::vector<Eigen::Matrix<T, SizeX, SizeY>> &matArray, int method = CONCAT_VERTICAL)
{
    Mat outMat;
    if (method == CONCAT_VERTICAL) {
        outMat.setZero(SizeX * matArray.size(), SizeY);
        int row = 0;
        for (auto &m : matArray) {
            outMat.block(row, 0, SizeX, SizeY) = m;
            row += SizeX;
        }
    } else {
        int fullWidth = 0;
        for (auto &m : matArray) fullWidth += m.cols();
        outMat.setZero(SizeX, fullWidth);
        int col = 0;
        for (auto &m : matArray) {
            outMat.block(col, 0, SizeX, m.cols()) = m;
            col += m.cols();
        }
    }
    return outMat;
}

int nbAvailableThreads();

std::pair<Vec2, Vec2> lineIntersectBox(const Vec3 &line, double w, double h /*, double x = 0.0, double y = 0.0*/);

void matchesMapsToTable(std::vector<std::map<int, int>> matchesMaps, Mati &table);
void findFullSubMeasurementMatrix(const Mat & W, Mat & Wfsub, std::vector<int> camIds = {},
                                  std::vector<int> * selectedCols = nullptr);
std::vector<std::vector<int>> generateBatches(int nbCams, int batchSize);

bool exportToPly(const Mat3Xf &vec_points_white, const std::string &sFileName,
                 const Mat3Xf &colors = Mat3Xf());

// [1] page(42) eq(2.17)
template <typename T>
Eigen::Matrix<T, 3, 3> RfromEulerZYZt(const T &t1, const T &rho, const T &t2)
{
    auto c0 = cos(t1);
    auto c1 = cos(rho);
    auto c2 = cos(t2);
    auto s0 = sin(t1);
    auto s1 = sin(rho);
    auto s2 = sin(t2);

    Eigen::Matrix<T, 3, 3> R;
    R(0, 0) = c0 * c1 * c2 + s0 * s2;
    R(0, 1) = c0 * c1 * s2 - s0 * c2;
    R(0, 2) = c0 * s1;
    R(1, 0) = s0 * c1 * c2 - c0 * s2;
    R(1, 1) = s0 * c1 * s2 + c0 * c2;
    R(1, 2) = s0 * s1;
    R(2, 0) = -s1 * c2;
    R(2, 1) = -s1 * s2;
    R(2, 2) = c1;
    return R;
}

template <typename T>
Eigen::Matrix<T, 3, 3> RfromEulerZYZt_inv(const T &theta1, const T &rho, const T &theta2)
{
    return RfromEulerZYZt(theta2, -rho, theta1);
}

template <typename T>
Eigen::Matrix<T, 3, 3> RfromEulerZYX(const T &rx, const T &ry, const T &rz)
{
    const auto cx = cos(rx);
    const auto cy = cos(ry);
    const auto cz = cos(rz);
    const auto sx = sin(rx);
    const auto sy = sin(ry);
    const auto sz = sin(rz);

    Eigen::Matrix<T, 3, 3> R;

    // R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
    //        cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
    //          -sy            cy*sx             cy*cx]
    //   = Rz(tz) * Ry(ty) * Rx(tx)

    R(0, 0) = cy*cz;
    R(0, 1) = sy*sx*cz - sz*cx;
    R(0, 2) = sy*cx*cz + sz*sx;
    R(1, 0) = cy*sz;
    R(1, 1) = sy*sx*sz + cz*cx;
    R(1, 2) = sy*cx*sz - cz*sx;
    R(2, 0) = -sy;
    R(2, 1) = cy*sx;
    R(2, 2) = cy*cx;

    return R;
}

template <typename T>
Eigen::Matrix<T, 3, 3> RfromEulerXYZ(const T &rx, const T &ry, const T &rz)
{
    const auto cx = cos(rx);
    const auto cy = cos(ry);
    const auto cz = cos(rz);
    const auto sx = sin(rx);
    const auto sy = sin(ry);
    const auto sz = sin(rz);

    Eigen::Matrix<T, 3, 3> R;

    // The rotation matrix R can be constructed as follows by
    // ct = [cx cy cz] and st = [sx sy sz]
    //
    // R = [            cy*cz,           -cy*sz,     sy]
    //     [ cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx]
    //     [ sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy]
    //   = Rx(tx) * Ry(ty) * Rz(tz)

    R(0,0) = cy*cz;
    R(0,1) = -cy*sz;
    R(0,2) = sy;
    R(1,0) = cx*sz + cz*sx*sy;
    R(1,1) = cx*cz - sx*sy*sz;
    R(1,2) = -cy*sx;
    R(2,0) = sx*sz - cx*cz*sy;
    R(2,1) = cz*sx + cx*sy*sz;
    R(2,2) = cx*cy;

    return R;
}

void EulerZYXfromR(const Mat3 &R, double &rx, double &ry, double &rz);
void EulerZYZtfromR(const Mat3 &R, double &t1, double &rho, double &t2);
void EulerZYZtfromRinv(const Mat3 &R, double &t1, double &rho, double &t2);

void wrapHalfPI(double &angleRad);

template <typename T, int SizeX, int SizeY>
Eigen::Matrix<T, SizeX, SizeY> inverseTransform(
    const Eigen::Matrix<T, SizeX, SizeY> &transform)
{
    // SizeX == SizeY;
    // transform.bottomRight(1, 1) == 1.0;
    Eigen::Matrix<T, SizeX - 1, SizeY - 1> R =
        transform.topLeftCorner(SizeX - 1, SizeY - 1);
    Eigen::Matrix<T, SizeX - 1, 1> t = transform.col(SizeY - 1).topRows(SizeX - 1);

    Eigen::Matrix<T, SizeX, SizeY> out;
    out.setIdentity(SizeX, SizeY);

    out.topLeftCorner(SizeX - 1, SizeY - 1) = R.transpose();
    out.rightCols(1).topRows(SizeX - 1) = -R.transpose() * t;
    return out;
}

// **** Math

Mat3 skewSym(const Vec3 &a);
double nullspace(const Eigen::Ref<const Mat> &A, Vec *nullsp);

Vec4 triangulate(const std::vector<Vec2> &x, const std::vector<Mat34> &Ps);

std::string getExecPath();
}  // namespace utils
}  // namespace p3d

#endif  // UTILS_H
