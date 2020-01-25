#include "gtest/gtest.h"

#include "test_paths.h"
#include "test_meta.h"

#include "p3d/core/serialization.h"

int main(int argc, char **argv) {

    impl::registerTypes();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
