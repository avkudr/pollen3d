#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core.h"
#include "p3d/stereo/dense_matching.h"

using namespace p3d;

TEST(DENSE, test_neighborInverse)
{
    Neighbor n;
    n.imLsize = cv::Size(15, 15);
    n.imRsize = cv::Size(16, 16);
    n.disp = cv::Mat::ones(3, 3, CV_32FC1);

    // qn = Trinv * (Tl * q - (disp / 16, 0, 0))
    // q  = Tlinv * (Tr * qn + disp )

    Neighbor ninv = n.inverse();

    EXPECT_EQ(ninv.imLsize, n.imRsize);
    EXPECT_EQ(ninv.imRsize, n.imLsize);
    EXPECT_TRUE(ninv.Trinv.isApprox(utils::inverseTransform(n.Tl)));
    EXPECT_TRUE(ninv.Tl.isApprox(utils::inverseTransform(n.Tr)));
    EXPECT_TRUE(utils::equalsCvMat(n.disp, -ninv.disp));
}

TEST(DENSE, test_mergeDenseMaps)
{
    std::map<int, std::map<int, Neighbor>> neighbors;

    int size = 2;

    // *** init some dummy neighbors

    neighbors[0][1].imLsize = cv::Size(size, size);
    neighbors[0][2].imLsize = cv::Size(size, size);
    neighbors[1][2].imLsize = cv::Size(size, size);
    neighbors[1][3].imLsize = cv::Size(size, size);
    neighbors[2][3].imLsize = cv::Size(size, size);

    neighbors[0][1].imRsize = cv::Size(size, size);
    neighbors[0][2].imRsize = cv::Size(size, size);
    neighbors[1][2].imRsize = cv::Size(size, size);
    neighbors[1][3].imRsize = cv::Size(size, size);
    neighbors[2][3].imRsize = cv::Size(size, size);

    neighbors[0][1].disp = cv::Mat::zeros(size, size, CV_32FC1);
    neighbors[0][2].disp = cv::Mat::zeros(size, size, CV_32FC1);
    neighbors[1][2].disp = cv::Mat::zeros(size, size, CV_32FC1);
    neighbors[1][3].disp = cv::Mat::zeros(size, size, CV_32FC1);
    neighbors[2][3].disp = cv::Mat::zeros(size, size, CV_32FC1);

    neighbors[0][1].confidence = cv::Mat::ones(size, size, CV_32FC1);
    neighbors[0][2].confidence = cv::Mat::ones(size, size, CV_32FC1);
    neighbors[1][2].confidence = cv::Mat::ones(size, size, CV_32FC1);
    neighbors[1][3].confidence = cv::Mat::ones(size, size, CV_32FC1);
    neighbors[2][3].confidence = cv::Mat::ones(size, size, CV_32FC1);

    for (auto& [camRef, map] : neighbors) {
        for (auto& [cam2, n] : map) {
            if (neighbors.count(cam2) > 0 && neighbors.at(cam2).count(camRef) > 0)
                continue;
            neighbors[cam2][camRef] = n.inverse();
        }
    }

    std::vector<MatchCandidate> landmark;
    // use this to print:
    //     for (const auto& l : landmark) l.print();

    // *** im0, point(0,0) : all disparities are correct
    landmark.clear();
    landmark = DenseMatchingUtil::findMatch(neighbors, 0, Vec2f(0.0, 0.0));
    EXPECT_EQ(landmark.size(), 4);
    for (const auto& l : landmark) EXPECT_FLOAT_EQ(l.confidence, 1.0f);

    // no disparity 0-1
    // but there is still another way => 0-2-1-3
    neighbors[0][1].disp.at<float>(0, 0) = DenseMatchingUtil::NO_DISPARITY;
    neighbors[1][0] = neighbors[0][1].inverse();

    landmark = DenseMatchingUtil::findMatch(neighbors, 0, Vec2f(0.0, 0.0));
    EXPECT_EQ(landmark.size(), 4);
    for (const auto& l : landmark) EXPECT_FLOAT_EQ(l.confidence, 1.0f);

    // no disparity still 0-1
    // but there is still a way => 0-2-1-3 but the confidence is lower
    // however, there is another better way 0-2-3-1 :)
    neighbors[1][2].confidence.at<float>(0, 0) = 0.8;
    neighbors[2][1] = neighbors[1][2].inverse();
    landmark = DenseMatchingUtil::findMatch(neighbors, 0, Vec2f(0.0, 0.0));
    EXPECT_EQ(landmark.size(), 4);
    for (const auto& l : landmark) EXPECT_FLOAT_EQ(l.confidence, 1.0f);

    // lower confidence for 1-3 path
    // the best is now the path through 0-2-1-3
    neighbors[1][3].confidence.at<float>(0, 0) = 0.7;
    neighbors[3][1] = neighbors[1][3].inverse();
    landmark = DenseMatchingUtil::findMatch(neighbors, 0, Vec2f(0.0, 0.0));

    EXPECT_EQ(landmark.size(), 4);
    std::sort(landmark.begin(), landmark.end());
    EXPECT_FLOAT_EQ(landmark[0].confidence, 1.0f);
    EXPECT_FLOAT_EQ(landmark[1].confidence, 0.7f);  // must be 0.8 !!!
    EXPECT_FLOAT_EQ(landmark[2].confidence, 1.0f);
    EXPECT_FLOAT_EQ(landmark[3].confidence, 1.0f);

    // delete the 3rd disparity
    neighbors[1][3].disp.at<float>(0, 0) = DenseMatchingUtil::NO_DISPARITY;
    neighbors[2][3].disp.at<float>(0, 0) = DenseMatchingUtil::NO_DISPARITY;
    neighbors[3][1] = neighbors[1][3].inverse();
    neighbors[3][2] = neighbors[2][3].inverse();
    landmark = DenseMatchingUtil::findMatch(neighbors, 0, Vec2f(0.0, 0.0));

    EXPECT_EQ(landmark.size(), 3);
    std::sort(landmark.begin(), landmark.end());
    EXPECT_FLOAT_EQ(landmark[0].confidence, 1.0f);
    EXPECT_FLOAT_EQ(landmark[1].confidence, 0.8f);
    EXPECT_FLOAT_EQ(landmark[2].confidence, 1.0f);
}
