#Requires -RunAsAdministrator
param (
  [switch]$s=$false,
  [switch]$h=$false,
  [string]$d="Release",
  [string]$c="main",
  [int]$j=[int]4
)

# Default values for variables.
$DEBUG_TYPE="Release"
$NUM_JOBS=4
$MOCO="on"
$CORE_BRANCH="main"

function Help {
    Write-Output "This script builds the last available version of OpenSim-Gui in your computer."
    Write-Output "Usage: ./scriptName [OPTION]..."
    Write-Output "Example: ./opensim-gui-build.sh -j4 -dRelease"
    Write-Output "    -d        Debug Type. Available Options:"
    Write-Output "                  Release (Default): No debugger symbols. Optimized."
    Write-Output "                  Debug: Debugger symbols. No optimizations (>10x slower). Library names ending with _d."
    Write-Output "                  RelWithDebInfo: Debugger symbols. Optimized. Bigger than Release, but not slower."
    Write-Output "                  MinSizeRel: No debugger symbols. Minimum size. Optimized."
    Write-Output "    -j        Number of jobs to use when building libraries (>=1)."
    Write-Output "    -s        Simple build without moco (Tropter and Casadi disabled)."
    Write-Output "    -c        Branch for opensim-core repository."
    Write-Output ""
    exit
}

# Get flag values if exist.
if ($h) {
    Help
}
if ($s) {
    $MOCO="off"
}
if ($d -ne "Release" -and $d -ne "Debug" -and $d -ne "RelWithDebInfo" -and $d -ne "MinSizeRel") {
    Write-Error "Value for parameter -d not valid."
} else {
    $DEBUG_TYPE=$d
}
if ($c) {
    $CORE_BRANCH=$c
}
if ($j -lt [int]1) {
    Write-Error "Value for parameter -j not valid."
    Help
} else {
    $NUM_JOBS=$j
}

Write-Output "DEBUG_TYPE $DEBUG_TYPE"
Write-Output "NUM_JOBS $NUM_JOBS"
Write-Output "MOCO $MOCO"
Write-Output "CORE_BRANCH $CORE_BRANCH"

# Install chocolatey
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# Install Microsoft Visual Studio 2022 Community
choco install visualstudio2022community -y
choco install visualstudio2022-workload-nativedesktop -y
choco install visualstudio2022buildtools -y

# Install cmake 3.23.2
choco install cmake.install --version 3.23.3 --installargs '"ADD_CMAKE_TO_PATH=System"' -y

# Install git
choco install git.install -y

# Install dependencies of opensim-core
choco install python3  -y
choco install jdk8  -y
choco install swig  -y
choco install nsis  -y
py -m pip install numpy

# Clone opensim-core
chdir C:/opensim-workspace/
&"$Env:Programfiles\Git\bin\git.exe" clone https://github.com/opensim-org/opensim-core.git C:/opensim-workspace/opensim-core-source
chdir C:/opensim-workspace/opensim-core-source
&"$Env:Programfiles\Git\bin\git.exe" checkout $CORE_BRANCH

# Generate dependencies project and build dependencies using superbuild
md C:/opensim-workspace/opensim-core-dependencies-build
chdir C:/opensim-workspace/opensim-core-dependencies-build
&"$Env:Programfiles\CMake\bin\cmake.exe" C:/opensim-workspace/opensim-core-source/dependencies/ -G"Visual Studio 17 2022" -A x64 -DCMAKE_INSTALL_PREFIX="C:/opensim-workspace/opensim-core-dependencies-install" -DSUPERBUILD_ezc3d:BOOL=on -DOPENSIM_WITH_CASADI:BOOL=$MOCO -DOPENSIM_WITH_TROPTER:BOOL=$MOCO
&"$Env:Programfiles\CMake\bin\cmake.exe" . -LAH
&"$Env:Programfiles\CMake\bin\cmake.exe" --build . --config $DEBUG_TYPE -- /maxcpucount:$NUM_JOBS

# Generate opensim-core build and build it
md C:/opensim-workspace/opensim-core-build
chdir C:/opensim-workspace/opensim-core-build
$env:CXXFLAGS = "/W0"
&"$Env:Programfiles\CMake\bin\cmake.exe" C:/opensim-workspace/opensim-core-source/ -G"Visual Studio 17 2022" -A x64 -DOPENSIM_DEPENDENCIES_DIR="C:/opensim-workspace/opensim-core-dependencies-install" -DBUILD_JAVA_WRAPPING=on -DBUILD_PYTHON_WRAPPING=on -DOPENSIM_C3D_PARSER=ezc3d -DBUILD_TESTING=off -DCMAKE_INSTALL_PREFIX="C:/opensim-core" -DOPENSIM_WITH_CASADI:BOOL=$MOCO -DOPENSIM_WITH_TROPTER:BOOL=$MOCO
&"$Env:Programfiles\CMake\bin\cmake.exe" . -LAH
&"$Env:Programfiles\CMake\bin\cmake.exe" --build . --config $DEBUG_TYPE -- /maxcpucount:$NUM_JOBS
&"$Env:Programfiles\CMake\bin\cmake.exe" --install .

# Test opensim-core
&"$Env:Programfiles\CMake\bin\ctest.exe" --parallel $NUM_JOBS --build-config $DEBUG_TYPE -E python*

# Test python bindings
cd C:\opensim-core\sdk\python
$PYTHON_PATH=(python -c "import os, sys; print(os.path.dirname(sys.executable))")
&"$PYTHON_PATH\python.exe" setup_win_python38.py
py -m unittest discover --start-directory opensim/tests --verbose