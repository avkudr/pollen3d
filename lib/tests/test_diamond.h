#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core.h"
#include "p3d/project.h"
#include "p3d/multiview/autocalib.h"
#include "p3d/tasks.h"
#include "p3d/utils.h"

#include <Eigen/Eigen>

using namespace p3d;

static void initDiamondProject(Project & project) {
    project = Project();

    Mat vertices;
    vertices.setZero(22, 4);
    vertices << 0, -147.20, 75.20, 1.0,
        0, -16.00, 303.20, 1.0,
        -119.20, -100.80, 162.40, 1.0,
        118.40, -100.80, 162.40, 1.0,
        134.40, -146.40, 1.60, 1.0,
        263.20, -98.40, 0, 1.0,
        268.80, -16.00, 160.00, 1.0,
        142.40, -16.00, 269.60, 1.0,
        349.60, -16.00, 23.20, 1.0,
        268.80, 0, 160.00, 1.0,
        142.40, 0, 269.60, 1.0,
        349.60, 0, 23.20, 1.0,
        0, 0, 303.20, 1.0,
        0, 320.00, 0, 1.0,
        -349.60, 0, 23.20, 1.0,
        -142.40, 0, 269.60, 1.0,
        -268.80, 0, 160.00, 1.0,
        -349.60, -16.00, 23.20, 1.0,
        -142.40, -16.00, 269.60, 1.0,
        -268.80, -16.00, 160.00, 1.0,
        -263.20, -98.40, 0, 1.0,
        -134.40, -146.40, 1.60, 1.0;
    vertices.transposeInPlace();

    project.pointCloudCtnr()["sparse"].setVertices(vertices.topRows(3).cast<float>());

    AffineCamera c;
    Image i1;
    i1.setCamera(c);
    i1.setTranslation({500,250});
    project.setImageList({i1,i1,i1,i1});

    project.imagePair(0)->setTheta1(utils::deg2rad(-42.0));
    project.imagePair(0)->setRho(utils::deg2rad(8.0));
    project.imagePair(0)->setTheta2(utils::deg2rad(-24.0));

    project.imagePair(1)->setTheta1(utils::deg2rad(4.0));
    project.imagePair(1)->setRho(utils::deg2rad(13.0));
    project.imagePair(1)->setTheta2(utils::deg2rad(10.0));

    project.imagePair(2)->setTheta1(utils::deg2rad(-32.0));
    project.imagePair(2)->setRho(utils::deg2rad(18.0));
    project.imagePair(2)->setTheta2(utils::deg2rad(-16.0));

    auto P = project.getCameraMatricesMat();
    auto W = P * vertices;
    for (int c = 0; c < project.nbImages(); c++) {
        auto lastRow = W.row(3*c + 2).cwiseInverse();
        W.row(3*c + 0).cwiseProduct(lastRow);
        W.row(3*c + 1).cwiseProduct(lastRow);
        W.row(3*c + 2).cwiseProduct(lastRow);
    }

    project.setMeasurementMatrix(W);
    project.setMeasurementMatrixFull(W);
}

/*
TEST(DIAMOND, draw)
{
    cv::namedWindow("image");
    cv::resizeWindow("image",1280,1024);
    cv::Mat im = cv::Mat(1024,1280,CV_8UC3);

    Project project;
    initDiamondProject(project);
    auto W = project.getMeasurementMatrix();

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
    Project project;
    initDiamondProject(project);

    auto W = project.getMeasurementMatrix();

    int id1 = 0;
    int id2 = 2;
    Mat2X pts1mat = W.middleRows(3*id1,2);
    Mat2X pts2mat = W.middleRows(3*id2,2);
    std::vector<Vec2> pts1, pts2;
    utils::convert(pts1mat, pts1);
    utils::convert(pts2mat, pts2);

    Mat3 F = FundMatUtil::findAffineCeres(pts1, pts2);

    double maxErrDistance = -1.0;
    double maxErrResidual = -1.0;

    for (int i = 0; i < pts1.size(); ++i) {
        // **** distance error
        Vec2 errs = FundMatUtil::epiporalDistancesF(F, pts1[i], pts2[i]);
        if (errs.norm() > maxErrDistance) maxErrDistance = errs.norm();

        // **** residual error
        Vec3 pt1H;
        pt1H << pts1[i], 1;
        Vec3 pt2H;
        pt2H << pts2[i], 1;
        auto errResidual = pt2H.transpose() * F * pt1H;
        if (abs(errResidual(0, 0)) > maxErrResidual) maxErrResidual = abs(errResidual(0, 0));
    }
    EXPECT_LT(maxErrDistance, 1e-5);
    EXPECT_LT(maxErrResidual, 1e-5);
}

TEST(DIAMOND, test2_slopAngles)
{
    Project project;
    initDiamondProject(project);
    auto W = project.getMeasurementMatrix();

    for (int id1 = 0; id1 < project.nbImages() - 1; ++id1) {
        int id2 = id1 + 1;
        Mat2X pts1mat = W.middleRows(3*id1,2);
        Mat2X pts2mat = W.middleRows(3*id2,2);
        std::vector<Vec2> pts1, pts2;
        utils::convert(pts1mat, pts1);
        utils::convert(pts2mat, pts2);

        Mat3 F = FundMatUtil::findAffineCeres(pts1, pts2);

        auto angles = FundMatUtil::slopAngles(F);
        EXPECT_FLOAT_EQ(angles.first, project.imagePair(id1)->getTheta1());
        EXPECT_FLOAT_EQ(angles.second, project.imagePair(id1)->getTheta2());
    }
}

TEST(DIAMOND, test3_autocalib)
{
    Project project;
    initDiamondProject(project);
    auto W = project.getMeasurementMatrix();
    MatX3 relRot = project.getCameraRotationsRelativeMat();
    Mat2X slopes(2,relRot.rows());
    slopes.row(0) = relRot.col(0);
    slopes.row(1) = relRot.col(2);

    AutoCalibrator autocalib(project.nbImages());

    autocalib.setMaxTime(60);

    autocalib.setMeasurementMatrix(W);
    autocalib.setSlopeAngles(slopes);
    autocalib.setStopValue(1e-10);
    autocalib.setFtolRel(1e-10);
    autocalib.setFtolAbs(1e-10);
    autocalib.setXtolRel(1e-10);
    autocalib.run();

    auto relRotCalib = autocalib.getRotationAngles();
    EXPECT_EQ(relRot.size(),relRotCalib.size());
    for (int u = 0; u < relRot.rows(); u++) {
        for (int v = 0; v < relRot.cols(); v++) {
            EXPECT_NEAR(relRot(u,v),
                        relRotCalib(u,v),
                        1e-3);
        }
    }
}

TEST(DIAMOND, test3_rotationDecomposition)
{
    Project project;
    initDiamondProject(project);

    std::vector<Mat3> R;
    project.getCamerasRotations(&R);

    for (auto i = 0; i < R.size() - 1; i++){
        Mat3 dR = R[i+1] * R[i].transpose();

        double t1, rho, t2;
        utils::EulerZYZtfromRinv(dR,t1,rho,t2);
        utils::wrapHalfPI(t1);
        utils::wrapHalfPI(t2);

        EXPECT_FLOAT_EQ(t1 , project.imagePair(i)->getTheta1());
        EXPECT_FLOAT_EQ(rho, project.imagePair(i)->getRho());
        EXPECT_FLOAT_EQ(t2 , project.imagePair(i)->getTheta2());
    }
}
