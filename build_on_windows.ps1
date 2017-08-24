# This is a PowerShell script to build Muscollo and its dependencies on
# Windows using Visual Studio.
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
    -DCMAKE_PREFIX_PATH="$pwd\..\muscollo_dependencies_install\eigen;$pwd\..\muscollo_dependencies_install\opensim_core"
cmake --build . --config RelWithDebInfo -- /maxcpucount:8
ctest --build-config RelWithDebInfo --parallel 8
cmake --build . --config RelWithDebInfo --target install -- /maxcpucount:8
