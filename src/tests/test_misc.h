#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/data/point_cloud_container.h"
#include "p3d/data/project_data.h"
#include "p3d/multiview/bundle_params.h"
#include "p3d/project_manager.h"
#include "p3d/utils.h"

using namespace p3d;

TEST(MISC, test_matchesToTable)
{
    std::vector<std::map<int, int>> matchesMaps;
    matchesMaps.resize(3);
    matchesMaps[0] = {{0,1},{1,2},{2,4},{3,5}};
    matchesMaps[1] = {{1,2},{2,6},{3,3},{5,5}};
    matchesMaps[2] = {{2,1},{3,4},{4,8},{6,2}};

    Mati tableTrue;
    tableTrue.setOnes(4,6);
    tableTrue << 0,  1,  2,  3, -1, -1,
                 1,  2,  4,  5,  3, -1,
                 2,  6, -1,  5,  3,  4,
                 1,  2, -1, -1,  4,  8;

    Mati table;
    utils::matchesMapsToTable(matchesMaps, table);

    EXPECT_EQ(table.rows(),4);
    EXPECT_EQ(table.cols(),6);
    EXPECT_EQ(table,tableTrue);
}

TEST(MISC, test_bundleParams)
{
    const int nbCams = 3;
    BundleParams p(nbCams);

    EXPECT_EQ(p.isConst(p3dBundleParam_K,0),false);
    EXPECT_EQ(p.isConst(p3dBundleParam_K,1),false);
    EXPECT_EQ(p.isConst(p3dBundleParam_K,2),false);

    p.setConstAllCams(p3dBundleParam_K);

    EXPECT_EQ(p.isConst(p3dBundleParam_K,0),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_K,1),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_K,2),true);

    EXPECT_EQ(p.isConst(p3dBundleParam_Alpha,1),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_Skew,1),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_R,1),false);

    p.setVarying(p3dBundleParam_R,{0});
    EXPECT_EQ(p.isVarying(p3dBundleParam_R,0),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_R,0),false);

    p.setConst(p3dBundleParam_t,{1,2});
    EXPECT_EQ(p.isVarying(p3dBundleParam_t,0),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_t,1),true);
    EXPECT_EQ(p.isConst(p3dBundleParam_t,2),true);

    p.setConstPts();
    EXPECT_EQ(p.isConstPts(),true);
    p.setVaryingPts();
    EXPECT_EQ(p.isConstPts(),false);

}

TEST(MISC, test_affineCamera)
{
    AffineCamera c;
    EXPECT_TRUE(c.getA().isApprox(Mat2::Identity()));

    Mat2 c1;
    c1 << 2.2, -0.005, 0.0, 2.0;
    c.setAlpha(1.1);
    c.setFocal(2.0);
    c.setSkew(-0.005);
    EXPECT_TRUE(c.getA().isApprox(c1));
}

TEST(MISC, test_rotations)
{
    auto random = []() {
        return (double)M_PI * rand() / RAND_MAX - M_PI / 2.0;
    };

    for (int i = 0; i < 100; i++) {
        double a = random(), b = random(), c = random();

        double a1, b1, c1;
        auto R1 = utils::RfromEulerZYZt(a, b, c);
        {
            utils::EulerZYZtfromR(R1, a1, b1, c1);
            EXPECT_NEAR(a, a1, 1e-5);
            EXPECT_NEAR(b, b1, 1e-5);
            EXPECT_NEAR(c, c1, 1e-5);
        }
        {
            auto R1inv = utils::RfromEulerZYZt_inv(a, b, c);
            EXPECT_TRUE(R1inv.isApprox(R1.inverse()));
        }
    }

    //    EXPECT_EQ(table.rows(), 4);
    //    EXPECT_EQ(table.cols(), 6);
    //    EXPECT_EQ(table, tableTrue);
}

TEST(MISC, test_pointCloudCtnr)
{
    PointCloudContainer ctnr;

    EXPECT_EQ(ctnr.contains("1"), false);

    try {
        ctnr.at("1");
        FAIL() << "throwException() should throw an error\n";
    } catch (const std::exception& e) {
        EXPECT_EQ(1, 1);
    }

    EXPECT_EQ(ctnr.size(), 0);
    ctnr["1"].setVertices(Mat3Xf::Random(3, 5));  // inserts a new
    EXPECT_EQ(ctnr.size(), 1);
    auto p1 = ctnr["1"];  // gives the existing
    EXPECT_EQ(ctnr.size(), 1);
    ctnr.erase("1");
    EXPECT_EQ(ctnr.size(), 0);
    ctnr["2"].setVertices(Mat3Xf::Random(3, 5));  // inserts a new
    ctnr["3"].setVertices(Mat3Xf::Random(3, 6));  // inserts a new
    ctnr["4"].setVertices(Mat3Xf::Random(3, 8));  // inserts a new
    EXPECT_EQ(ctnr.size(), 3);
    auto p2 = ctnr.at("2");
    EXPECT_EQ(p2.nbPoints(), 5);
}

TEST(MISC, test_pointCloudCtnrCmds)
{
    PointCloudContainer ctnr;

    {
        auto& pcd = ctnr["random"];
        int nbPts = 10;
        Mat3Xf verts = Mat3Xf::Random(3, nbPts);
        CommandManager::get()->executeCommand(new CommandSetProperty(
            &pcd, P3D_ID_TYPE(p3dPointCloud_vertices), verts, true));

        EXPECT_EQ(pcd.nbPoints(), nbPts);
        EXPECT_EQ(ctnr["random"].nbPoints(), nbPts);
    }

    {
        auto size = ctnr.size();

        auto label = "hello";
        Mat3Xf verts2 = Mat3Xf::Random(3, 23);
        CommandManager::get()->executeCommand(
            new CommandPointCloudAdd(&ctnr, label, verts2));

        EXPECT_TRUE(ctnr.contains(label));
        EXPECT_EQ(ctnr[label].getVertices(), verts2);

        CommandManager::get()->undoCommand();

        EXPECT_TRUE(!ctnr.contains(label));

        ctnr[label].setVertices(verts2);
        size = ctnr.size();
        CommandManager::get()->executeCommand(
            new CommandPointCloudDelete(&ctnr, label));

        EXPECT_TRUE(!ctnr.contains(label));
        EXPECT_EQ(ctnr.size(), size - 1);

        CommandManager::get()->undoCommand();

        EXPECT_TRUE(ctnr.contains(label));
        EXPECT_EQ(ctnr[label].getVertices(), verts2);
    }
}
