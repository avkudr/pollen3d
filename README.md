# ![pollen3d Logo](app/assets/pollen3d_icon64.png) pollen3d

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
git clone --recursive https://github.com/avkudr/pollen3d
cd pollen3d

# install dependencies if needed
mkdir 3rdparty_release
cd 3rdparty_release
conan remote add bincrafters "https://api.bintray.com/conan/bincrafters/public-conan"
conan remote add camposs "https://conan.campar.in.tum.de/api/conan/conan-camposs"
#unix
conan install .. -s build_type=Release --build=missing -s compiler.libcxx=libstdc++11
#windows
conan install .. -s build_type=Release --build=missing -s compiler.cppstd=14 -s compiler.version=15
cd ..

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
./app/bin/pollen3d
```

## References

If you use pollen3d in your research activities, please cite [this paper](https://tel.archives-ouvertes.fr/tel-01930234/document)
```
@phdthesis{kudryavtsev20173d,
  title={3D Reconstruction in Scanning Electron Microscope: from image acquisition to dense point cloud},
  author={Kudryavtsev, Andrey},
  year={2017},
  school={University of Bourgogne Franche-Comt{\'e}}
}
```
