#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core.h"
#include "p3d/project.h"
#include "p3d/tasks.h"
#include "p3d/utils.h"

#ifndef P3D_PROJECT_EXTENSION
#define P3D_PROJECT_EXTENSION ".yml.gz"
#endif

TEST(PATHS, path_baseName)
{
    std::string basename = "path_to.yml.gz";
    ASSERT_EQ(basename,
              p3d::utils::baseNameFromPath("/home/andrey/" + basename));
    ASSERT_EQ(basename,
              p3d::utils::baseNameFromPath("//home//andrey//" + basename));
    ASSERT_EQ(basename,
              p3d::utils::baseNameFromPath("C:/dev/dqsdqsdqs/" + basename));
    ASSERT_EQ(basename,
              p3d::utils::baseNameFromPath("C:\\dev\\dqsdqsdqs\\" + basename));
}

TEST(PATHS, path_endsWith)
{
    std::string basename = "path_to" + std::string(P3D_PROJECT_EXTENSION);
    ASSERT_TRUE(p3d::utils::endsWith(basename, ".yml.gz"));
    ASSERT_TRUE(p3d::utils::endsWith(basename, "gz"));
    ASSERT_FALSE(p3d::utils::endsWith(basename, "..gz"));
}
