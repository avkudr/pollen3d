#include "gtest/gtest.h"

#include "test_paths.h"
#include "test_meta.h"
#include "test_misc.h"
#include "test_fundmat.h"
#include "test_diamond.h"

#include "p3d/serialization.h"

int main(int argc, char **argv)
{
//#ifndef POLLEN3D_DEBUG
//    std::cout.setstate(std::ios_base::failbit);
//    std::cerr.setstate(std::ios_base::failbit);
//#endif

    p3d::initStdLoger();

    p3d::ProjectData p1;
    p3d::ProjectData p2;

    p3d::Image im1("Brassica01.jpg");
    p3d::Image im2("Brassica02.jpg");

    p1.setImageList({im1,im2});
    ProjectManager::get()->loadImages(&p2,{"Brassica01.jpg","Brassica02.jpg"});

    std::cout << p1.nbImages() << std::endl;
    std::cout << p2.nbImages() << std::endl;

    //ProjectManager::get()->extractFeatures(p1,{});
    ProjectManager::get()->extractFeatures(p2,{});

    return 0;
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
}
