# This is a PowerShell script that performs the following steps:
# 1. Obtain/build Muscollo's dependencies.
# 2. Build Muscollo.
# 3. Test Muscollo.
# 4. Install Muscollo.
# The script will make multiple directories adjacent to the directory
# containing this script.
mkdir ..\muscollo_dependencies_build
cd ..\muscollo_dependencies_build
cmake ..\muscollo\dependencies `
    -G"Visual Studio 14 2015 Win64"
cmake --build . --config RelWithDebInfo -- /maxcpucount:8
mkdir ..\build
cd ..\build
cmake ..\muscollo `
    -G"Visual Studio 14 2015 Win64" `
    -DCMAKE_INSTALL_PREFIX="..\muscollo_install" `
    -DADOLC_DIR="..\muscollo_dependencies_install\adol-c" `
    -DIPOPT_DIR="..\muscollo_dependencies_install\ipopt" `
    -DCMAKE_PREFIX_PATH="$pwd\..\muscollo_dependencies_install\eigen;$pwd\..\muscollo_dependencies_install\opensim-core;$pwd\..\muscollo_dependencies_install\colpack"
cmake --build . --config RelWithDebInfo -- /maxcpucount:8
ctest --build-config RelWithDebInfo --parallel 8
cmake --build . --config RelWithDebInfo --target install -- /maxcpucount:8
