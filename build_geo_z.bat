@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >/dev/null 2>&1
cd /d E:\Git_Repository\curved-slicer
cmake --build build_nmake --target geometric_z_bc_unit_test 2>&1
echo BUILD_EXIT=%ERRORLEVEL%
