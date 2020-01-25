#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/core.h"

class Matcher
{
public:
    Matcher();
    Matcher(int method);
    ~Matcher();

    int _method = METHOD_BruteForceL1; //one of methods from enum
    double _filterCoef = 0.3;
    void setFilterCoef(double filterCoef);
    void setMethod(int method);
    void init(cv::Mat & leftDesc,
              cv::Mat & rightDesc,
              std::vector<cv::KeyPoint> & _keyptsLeft,
              std::vector<cv::KeyPoint> & _keyptsRight);

    void setDescriptors(cv::Mat & left, cv::Mat & right);

    bool isInit();

    void calculateMatches();

    int getMatchesNb();
    std::vector<cv::DMatch> getMatchesAsDMatchVector();
    cv::Mat _matchesTable;
    cv::Mat getMatchesAsIdxTable();
    cv::Mat getMatchesAsIdxColumn();
    cv::Mat getMatchesAsMeasMatrix();
    void  setMatchesTable(const cv::Mat & table);
    enum methods{
        METHOD_BruteForceL1,          ///< Perform linear interpolation on the table
        METHOD_BruteForce,          ///< Perform parabolic interpolation on the table
        METHOD_FlannBased           ///< Perform parabolic interpolation on the table
    };

    std::vector<cv::Point2d> _inliersLeft;
    std::vector<cv::Point2d> _inliersRight;

    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

private:
    const std::string* _methodsStrings = new std::string[3] {"BruteForce-L1","BruteForce","FlannBased"};
    std::vector<cv::DMatch> _matches;

    cv::Mat * _descriptorsLeftImage = NULL;
    cv::Mat * _descriptorsRightImage = NULL;

    std::vector<cv::KeyPoint> * _keypointsLeftImage = NULL;
    std::vector<cv::KeyPoint> * _keypointsRightImage = NULL;

public:
    std::vector<cv::Point2d> getInliersLeftImage() { return getInliers(0); }
    std::vector<cv::Point2d> getInliersRightImage() { return getInliers(1); }
    std::vector<cv::Point2d> getInliers(int imageIdx);
};

static void read(const cv::FileNode& node, Matcher& x, const Matcher& default_value = Matcher()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}
