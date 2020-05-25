#include "gtest/gtest.h"

#include "test_dense.h"
#include "test_diamond.h"
#include "test_fundmat.h"
#include "test_meta.h"
#include "test_misc.h"
#include "test_paths.h"

#include "p3d/serialization.h"

int main(int argc, char **argv)
{
    // example command
    // ./p3d_tests DENSE.test_neighborInverse -s

    cv::String keys =
        "{@test    |  | selected test to run}"
        "{silent s |  | silent std output}"
        "{help     |  | show help message}"
        "\n";

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    std::string test = parser.get<cv::String>(0);
    if (parser.has("s")) {
        p3d::logger::setStd();
        p3d::logger::off();
        std::cout.setstate(std::ios_base::failbit);
        std::cerr.setstate(std::ios_base::failbit);
    }

    std::string testSelection = parser.get<cv::String>(0);

    ::testing::InitGoogleTest(&argc, argv);
    if (testSelection != "") ::testing::GTEST_FLAG(filter) = testSelection;

    return RUN_ALL_TESTS();
}
