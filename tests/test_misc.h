#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core.h"
#include "p3d/data/affine_camera.h"
#include "p3d/data/project_data.h"
#include "p3d/multiview/bundle_params.h"
#include "p3d/project_manager.h"
#include "p3d/utils.h"

using namespace p3d;

TEST(MISC, test_matchesToTable)
{
    std::vector<std::map<int,int>> matchesMaps;
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

