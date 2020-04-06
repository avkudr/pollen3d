# ![pollen3d Logo](assets/pollen3d_icon64.png) pollen3d

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

pollen3d is available for Windows, MacOS, and Linux.

## Install

* install [cmake](https://cmake.org/download/)
* check that cmake is installed by running ```cmake --version``` in terminal (or cmd.exe)
the output should be similar to ```cmake version 3.13.4```
* install [conan](https://docs.conan.io/en/latest/installation.html) (c++ software apckage manager)
* check that conan is installed by running ```conan --version``` in terminal (or cmd.exe)
the output should be similar to ```Conan version 1.23.0```
* run the following commands in the terminal of your choice
```bash
conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan
mkdir build && cd build
cmake ..
cmake --build . -j 4
```

## References
