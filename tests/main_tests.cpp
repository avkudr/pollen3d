#include "gtest/gtest.h"

#include "test_paths.h"
#include "test_meta.h"
#include "test_misc.h"
#include "test_fundmat.h"
#include "test_diamond.h"

#include "p3d/core/serialization.h"

int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
