#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core/utils.h"
#include "p3d/core/core.h"
#include "p3d/project_manager.h"
#include "p3d/data/project_data.h"

TEST(FUNDMAT, fundmat_testImages)
{
    ProjectData data;
    ProjectSettings settings;
    settings.matcherFilterCoef = 0.3f;

    ProjectManager::get()->loadImages(&data,{
            "../../_datasets/brassica/Brassica01.jpg",
            "../../_datasets/brassica/Brassica02.jpg"
        });

    // --- extract features from all images. {} = all
    ProjectManager::get()->extractFeatures(data,{});

    EXPECT_GE(data.image(0)->getNbFeatures(), 800);
    EXPECT_GE(data.image(1)->getNbFeatures(), 800);

    // --- extract matches ({} = for all pairs)
    ProjectManager::get()->matchFeatures(data,{});

    ProjectManager::get()->findFundamentalMatrix(data,{});

    Vec errors;
    data.getEpipolarErrors(0,errors);
    EXPECT_GE(0.5f, errors.mean());
}

