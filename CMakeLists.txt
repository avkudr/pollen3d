cmake_minimum_required(VERSION 3.1)
project( pollen3d_all )

OPTION(BUILD_UNIT_TESTS "Build unit tests" ON)

add_subdirectory(./lib)
#add_subdirectory(./tests)
#add_subdirectory(./gui)

#add_subdirectory(./src/tests)
