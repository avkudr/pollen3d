#include "gtest/gtest.h"

#include "test_paths.h"
#include "test_meta.h"
#include "test_misc.h"
#include "test_fundmat.h"
#include "test_diamond.h"

#include "p3d/serialization.h"

int main(int argc, char **argv)
{
#ifndef POLLEN3D_DEBUG
    std::cout.setstate(std::ios_base::failbit);
    std::cerr.setstate(std::ios_base::failbit);
#endif

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
