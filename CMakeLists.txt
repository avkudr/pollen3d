cmake_minimum_required(VERSION 3.1)
project( all )

OPTION(BUILD_UNIT_TESTS "Build unit tests" ON)

add_subdirectory(./src/pollen3d_library)
add_subdirectory(./src/pollen3d_gui)
add_subdirectory(./src/tests)

#add_subdirectory(./src/tests)
