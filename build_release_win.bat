@echo off

echo Pollen3D build: start
cd %~dp0
if exist build_release\NUL rd /s /q build_release
mkdir build_release
cd build_release

cmake ..
cmake --build . --config Release -j 4

ren bin pollen3d
7z a -tzip pollen3d.zip -r pollen3d
move pollen3d.zip ..

cd ..
rd /s /q build_release
echo Pollen3D build: done

