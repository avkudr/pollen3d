#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core/utils.h"
#include "p3d/core/core.h"
#include "p3d/project_manager.h"
#include "p3d/data/project_data.h"

#include <Eigen/Eigen>

using utils::deg2rad;
using utils::RfromEulerZYZ;

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

        K << 1,0,0,0,1,0,0,0,1;

        R.resize(4);
        R[0] = RfromEulerZYZ(           0.0,             0.0,            0.0);
        R[1] = RfromEulerZYZ(deg2rad( -45.0), deg2rad(  -8.0),deg2rad( -45.0));
        R[2] = RfromEulerZYZ(deg2rad(   0.0), deg2rad( -13.0),deg2rad(   0.0));
        R[3] = RfromEulerZYZ(deg2rad( -20.0), deg2rad( -18.0),deg2rad( -20.0));

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
};

TEST(DIAMOND, test_fundMat)
{
    Diamond d;
    auto W = d.W();

    int id1 = 1;
    int id2 = 3;
    std::vector<Vec2> pts1;
    std::vector<Vec2> pts2;
    for (int i = 0; i < W.cols(); i++){
        pts1.emplace_back(W.block(3*id1,i,2,1));
        pts2.emplace_back(W.block(3*id2,i,2,1));
    }
    Mat3 Fe = fundmat::findFundMatCeres(pts1,pts2);
    Mat3 F  = fundmat::affineFromP(d.P[id1],d.P[id2]);

    EXPECT_TRUE(Fe.isApprox(F,1e-5));

    // x2' * F * x1
    auto residual = (W.block(3*id2,0,3,1).transpose() * F * W.block(3*id1,0,3,1));
    EXPECT_GE(1e-10,residual(0,0));
}
