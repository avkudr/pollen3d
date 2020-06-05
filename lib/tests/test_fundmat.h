#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core.h"
#include "p3d/project.h"
#include "p3d/tasks.h"
#include "p3d/utils.h"

using namespace p3d;

TEST(FUNDMAT, fundmat_testData)
{
    // Init some point pairs
    double xL[21] = {437.9474182128906, 685.6380004882813, 673.5831909179688,
                     533.0491333007813, 319.1584777832031, 483.3897399902344,
                     333.6655883789063, 476.6916809082031, 269.8779907226563,
                     429.8667602539063, 216.3757171630859, 372.67333984375,
                     304.4205932617188, 319.5361328125,    335.4749145507813,
                     382.3995666503906, 572.8048095703125, 245.6207733154297,
                     707.018310546875,  171.2962799072266, 420.7103271484375};
    double yL[21] = {240.1227111816406, 272.74755859375,   382.7117614746094,
                     514.738525390625,  461.6735229492188, 473.9537658691406,
                     217.2646789550781, 251.5523071289063, 272.0774536132813,
                     343.7459106445313, 265.4274291992188, 450.608154296875,
                     368.2933959960938, 371.5750732421875, 397.6313781738281,
                     493.9369201660156, 518.26904296875,   521.64794921875,
                     525.8312377929688, 492.1295776367188, 499.5015869140625};
    double xR[21] = {439.6264343261719, 690.5702514648438, 674.76904296875,
                     531.8222045898438, 317.2567749023438, 481.4939270019531,
                     337.3369750976563, 477.8121643066406, 272.2184753417969,
                     428.4476013183594, 219.7380218505859, 370.6543579101563,
                     302.905517578125,  318.7903137207031, 334.0060424804688,
                     379.8451232910156, 571.9124755859375, 245.0708465576172,
                     710.9512939453125, 171.4187927246094, 418.00390625};
    double yR[21] = {228.8862152099609, 261.2438354492188, 371.1325988769531,
                     501.7744140625,    449.7481689453125, 461.4763793945313,
                     206.1365509033203, 240.0611267089844, 260.6071472167969,
                     331.8235473632813, 253.3300323486328, 438.400146484375,
                     356.3855285644531, 359.8466796875,    385.4632568359375,
                     480.7957763671875, 505.9399719238281, 509.1502075195313,
                     512.67431640625,   479.6252746582031, 487.0714111328125};

    std::vector<Vec2> ptsL;
    std::vector<Vec2> ptsR;
    for (int i = 0; i < 21; i++) {
        ptsL.emplace_back(Vec2(xL[i], yL[i]));
        ptsR.emplace_back(Vec2(xR[i], yR[i]));
    }

    Mat3 F = FundMatUtil::findAffineCeres(ptsL, ptsR);

    Mat2X distances;
    distances.setZero(2, ptsL.size());
    for (int i = 0; i < ptsL.size(); ++i) {
        Vec2 errs = FundMatUtil::epiporalDistancesF(F, ptsL[i], ptsR[i]);
        distances.col(i) = errs;
    }
    EXPECT_GE(2.0, distances.mean());
}

TEST(MISC, images_Matching)
{
    p3d::Project data;
    p3d::loadImages(data, {std::string(P3D_IMAGES_DIR) + "/brassica/Brassica01.jpg",
                           std::string(P3D_IMAGES_DIR) + "/brassica/Brassica02.jpg"});

    data.settings()->matchingFilterCoeff = 0.2f;

    p3d::extractFeatures(data);

    EXPECT_EQ(data.nbImages(), 2);
    EXPECT_NE(data.image(0), nullptr);
    EXPECT_NE(data.image(1), nullptr);
    EXPECT_GE(data.image(0)->getNbFeatures(), 100);
    EXPECT_GE(data.image(1)->getNbFeatures(), 100);

    p3d::matchFeatures(data);
    EXPECT_GE(data.imagePairs()[0][1].nbMatches(), 100);
}

TEST(MISC, images_FundMat)
{
    p3d::Project data;
    p3d::loadImages(data, {std::string(P3D_IMAGES_DIR) + "/brassica/Brassica01.jpg",
                           std::string(P3D_IMAGES_DIR) + "/brassica/Brassica02.jpg"});

    data.settings()->matchingFilterCoeff = 0.2f;

    p3d::extractFeatures(data);
    p3d::matchFeatures(data);

    p3d::findFundamentalMatrix(data);

    Vec errors;
    data.getEpipolarErrorsResidual(0, 1, errors);
    Mat2X distances;
    data.getEpipolarErrorsDistance(0, 1, distances);
    EXPECT_GE(0.5f, errors.mean());
}
