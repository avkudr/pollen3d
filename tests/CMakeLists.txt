cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 17)

# Setup testing
enable_testing()
find_package(GTest REQUIRED)

if (GTest_FOUND)
    message("GoogleTest - FOUND")
endif()

include_directories(${GTEST_INCLUDE_DIR})

# Add test cpp file
add_executable(pollen3d_tests
    ./tests/test_paths.h
    ./tests/test_meta.h
    ./tests/test_fundmat.h
    ./tests/main_tests.cpp
    ${SOURCES}
)

# Link test executable against gtest & gtest_main
target_link_libraries(pollen3d_tests ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
target_link_libraries(pollen3d_tests ${TARGET_LIBS})

add_test(
    NAME pollen3d_tests
    COMMAND pollen3d_tests
)
