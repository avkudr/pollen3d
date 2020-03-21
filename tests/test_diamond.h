#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core/utils.h"
#include "p3d/core/core.h"
#include "p3d/core/autocalib.h"
#include "p3d/project_manager.h"
#include "p3d/data/project_data.h"

#include <Eigen/Eigen>

using utils::deg2rad;

struct Diamond {
    Diamond() {
        vertices.setZero(22,4);
        vertices << 0,-147.20,75.20, 1.0,
        0,-16.00,303.20, 1.0,
        -119.20,-100.80,162.40, 1.0,
        118.40,-100.80,162.40, 1.0,
        134.40,-146.40,1.60, 1.0,
        263.20,-98.40,0, 1.0,
        268.80,-16.00,160.00, 1.0,
        142.40,-16.00,269.60, 1.0,
        349.60,-16.00,23.20, 1.0,
        268.80,0,160.00, 1.0,
        142.40,0,269.60, 1.0,
        349.60,0,23.20, 1.0,
        0,0,303.20, 1.0,
        0,320.00,0, 1.0,
        -349.60,0,23.20, 1.0,
        -142.40,0,269.60, 1.0,
        -268.80,0,160.00, 1.0,
        -349.60,-16.00,23.20, 1.0,
        -142.40,-16.00,269.60, 1.0,
        -268.80,-16.00,160.00, 1.0,
        -263.20,-98.40,0, 1.0,
        -134.40,-146.40,1.60, 1.0;
        vertices.transposeInPlace();

        K << 1,0,220,0,1,200,0,0,1;

        slopes.resize(2,4);
        slopes << 0, -42.0,  4.0, -32.0,
                  0, -24.0, 10.0, -16.0;

        rhos = {0.0, 8.0, 13.0, 18.0};

        R.resize(4);
        R[0].setIdentity();
        R[1] = utils::RfromEulerZYZt_inv(deg2rad(slopes(0,1)), deg2rad(rhos[1]),deg2rad(slopes(1,1))) * R[0];
        R[2] = utils::RfromEulerZYZt_inv(deg2rad(slopes(0,2)), deg2rad(rhos[2]),deg2rad(slopes(1,2))) * R[1];
        R[3] = utils::RfromEulerZYZt_inv(deg2rad(slopes(0,3)), deg2rad(rhos[3]),deg2rad(slopes(1,3))) * R[2];

        t = std::vector<Vec2>(R.size(),{500,250});

        Mat34 Proj;
        Proj << 1,0,0,0,
                0,1,0,0,
                0,0,0,1;

        P.resize(R.size());
        for (auto i = 0; i < R.size(); ++i) {
            Mat4 T;
            T.setIdentity();
            T.block(0,0,3,3) = R[i];
            T.block(0,3,3,1) << t[i],0;
            P[i] = K * Proj * T;
        }
    }

    std::vector<Vec2> pts(int imId) {
        std::vector<Vec2> pts1;
        auto W_ = W();
        for (int i = 0; i < W_.cols(); i++){
            pts1.emplace_back(W_.block(3*imId,i,2,1));
        }
        return pts1;
    }

    int nbIm() { return static_cast<int>(R.size()); }
    Mat W() {
        Mat W;
        W.setZero(3*nbIm(),vertices.cols());
        for (int i = 0; i < nbIm(); ++i) {
            W.middleRows(3*i,3) = P[i] * vertices;
        }
        return W;
    }

    Mat vertices;
    Mat3 K;
    std::vector<Mat34> P;
    std::vector<Mat3> R;
    std::vector<Vec2> t;
    Mat2X slopes;
    std::vector<double> rhos;
};

/*
TEST(DIAMOND, draw)
{
    cv::namedWindow("image");
    cv::resizeWindow("image",1280,1024);
    cv::Mat im = cv::Mat(1024,1280,CV_8UC3);

    Diamond d;
    auto W = d.W();
    int id1 = 0;
    int id2 = 2;
    std::vector<Vec2> pts1;
    std::vector<Vec2> pts2;
    for (int i = 0; i < W.cols(); i++){
        Vec2 pt1 = W.block(3*id1,i,2,1);
        Vec2 pt2 = W.block(3*id2,i,2,1);

        cv::circle(im,cv::Point(pt1(0),pt1(1)),3,cv::Scalar(255,0,0));
        cv::circle(im,cv::Point(pt2(0),pt2(1)),3,cv::Scalar(0,255,0));
    }

    cv::imshow("image",im);
    cv::waitKeyEx(0);
}
*/

TEST(DIAMOND, test1_fundMatCeres)
{
    Diamond d;

    int id1 = 0;
    int id2 = 2;
    std::vector<Vec2> pts1 = d.pts(id1);
    std::vector<Vec2> pts2 = d.pts(id2);
    Mat3 F = fundmat::findAffineCeres(pts1,pts2);

    double maxErrDistance = -1.0;
    double maxErrResidual = -1.0;

    for (int i = 0; i < pts1.size(); ++i) {
        // **** distance error
        Vec2 errs = fundmat::epiporalDistancesF(F,pts1[i],pts2[i]);
        if (errs.norm() > maxErrDistance) maxErrDistance = errs.norm();

        // **** residual error
        Vec3 pt1H; pt1H << pts1[i], 1;
        Vec3 pt2H; pt2H << pts2[i], 1;
        auto errResidual = pt2H.transpose() * F * pt1H;
        if (abs(errResidual(0,0)) > maxErrResidual) maxErrResidual = abs(errResidual(0,0));
    }
    EXPECT_LT(maxErrDistance,1e-5);
    EXPECT_LT(maxErrResidual,1e-5);
}

TEST(DIAMOND, test2_slopAngles)
{
    Diamond d;
    for (int id1 = 0; id1 < d.nbIm() -1; ++id1) {
        int id2 = id1 + 1;
        std::vector<Vec2> pts1 = d.pts(id1);
        std::vector<Vec2> pts2 = d.pts(id2);
        Mat3 F = fundmat::findAffineCeres(pts1,pts2);

        auto angles = fundmat::slopAngles(F);
        EXPECT_FLOAT_EQ(utils::rad2deg(angles.first) , d.slopes(0,id2));
        EXPECT_FLOAT_EQ(utils::rad2deg(angles.second), d.slopes(1,id2));
    }
}

TEST(DIAMOND, test3_autocalib)
{
    std::cout.setstate(std::ios_base::failbit);

    Diamond d;
    auto W = d.W();

    AutoCalibrator autocalib(d.nbIm());

    autocalib.setMaxTime( 60 );

    autocalib.setMeasurementMatrix(W);
    autocalib.setSlopeAngles(utils::deg2rad(d.slopes));
    autocalib.run();

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "Calibration     :\n" << autocalib.getCalibrationMatrix().format(CleanFmt) << std::endl;
    std::cout << "Camera matrices :\n" << utils::concatenateMat(autocalib.getCameraMatrices()).format(CleanFmt) << std::endl;
    auto rot = utils::rad2deg(autocalib.getRotationAngles());
    std::cout << "Rotation angles :\n" << rot.format(CleanFmt) << std::endl;

    for (int i = 0; i < d.nbIm(); ++i){
        EXPECT_NEAR(rot(i,0),d.slopes(0,i),1e-3);
        EXPECT_NEAR(rot(i,1),d.rhos[i],1e-3);
        EXPECT_NEAR(rot(i,2),d.slopes(1,i),1e-3);
    }
}
