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

double rad2deg(double angRad);
double deg2rad(double angDeg);

void makeNonHomogenious(cv::Mat & m);
void copyMatElementsToVector(const cv::Mat & mat, const cv::Mat & idx, std::vector<double> & vec);
void copyVectorToMatElements(const std::vector<double> & vec, const cv::Mat & idx, cv::Mat & mat);

std::string baseNameFromPath(const std::string & path);
bool endsWith(std::string const & value, std::string const & ending);
std::vector<std::string> split(const std::string& str, const std::string& delim);

bool equalsCvMat(const cv::Mat & lhs, const cv::Mat & rhs);

enum{
    CONCAT_HORIZONTAL,
    CONCAT_VERTICAL,
};

cv::Mat concatenateMat(const std::vector<cv::Mat> & matArray, int method = CONCAT_VERTICAL);

int nbAvailableThreads();

std::pair<Vec2,Vec2> lineIntersectBox(const Vec3& line, double w, double h/*, double x = 0.0, double y = 0.0*/);

void matchesMapsToTable(std::vector<std::map<int, int> > matchesMaps, Mati &table);
}

#endif // UTILS_H
