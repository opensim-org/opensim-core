# This is a PowerShell script that performs the following steps:
# 1. Obtain/build Moco's dependencies.
# 2. Build Moco.
# 3. Test Moco.
# 4. Install Moco.
# The script will make multiple directories adjacent to the directory
# containing this script.
git submodule update --init
mkdir ..\moco_dependencies_build
cd ..\moco_dependencies_build
cmake ..\opensim-moco\dependencies `
    -G"Visual Studio 16 2019 Win64"
cmake --build . --config RelWithDebInfo -- /maxcpucount:8
mkdir ..\build
cd ..\build
cmake ..\opensim-moco `
    -G"Visual Studio 16 2019 Win64"
cmake --build . --config RelWithDebInfo -- /maxcpucount:8
ctest --build-config RelWithDebInfo --parallel 8
cmake --build . --config RelWithDebInfo --target install -- /maxcpucount:8
