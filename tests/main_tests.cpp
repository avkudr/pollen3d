#include "gtest/gtest.h"

#include "test_paths.h"
#include "test_meta.h"
#include "test_misc.h"
#include "test_fundmat.h"
#include "test_diamond.h"

#include "p3d/serialization.h"

int main(int argc, char **argv)
{
    std::cout.setstate(std::ios_base::failbit);
    std::cerr.setstate(std::ios_base::failbit);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
