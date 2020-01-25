#pragma once

#include <string>

#include "gtest/gtest.h"

#include "p3d/core/utils.h"
#include "p3d/core/core.h"
#include "p3d/project_manager.h"
#include "p3d/data/project_data.h"

#ifndef P3D_PROJECT_EXTENSION
#define P3D_PROJECT_EXTENSION ".yml.gz"
#endif

TEST(PATHS, baseNameFromPath)
{
    std::string basename = "path_to.yml.gz";
    ASSERT_EQ(basename, utils::baseNameFromPath("/home/andrey/" + basename));
    ASSERT_EQ(basename, utils::baseNameFromPath("//home//andrey//" + basename));
    ASSERT_EQ(basename, utils::baseNameFromPath("C:/dev/dqsdqsdqs/" + basename));
    ASSERT_EQ(basename, utils::baseNameFromPath("C:\\dev\\dqsdqsdqs\\" + basename));
}

TEST(PATHS, endsWith)
{
    std::string basename = "path_to" + std::string(P3D_PROJECT_EXTENSION);
    ASSERT_TRUE (utils::endsWith(basename,".yml.gz"));
    ASSERT_TRUE (utils::endsWith(basename,"gz"));
    ASSERT_FALSE(utils::endsWith(basename,"..gz"));
}

TEST(PATHS, saveOpenProjectString)
{
    ProjectData data;
    std::string path = "dummy_project" + std::string(P3D_PROJECT_EXTENSION);
    data.setProjectPath(path);
    ProjectManager::get()->saveProject(&data,path);

    ProjectData data2;
    ProjectManager::get()->openProject(&data2,path);

    EXPECT_EQ(data.getProjectPath(),data2.getProjectPath());
}

