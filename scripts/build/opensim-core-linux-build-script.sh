#!/bin/bash

# Exit when an error happens instead of continue.
set -e

# Default values for flags.
DEBUG_TYPE="Release"
NUM_JOBS=4
MOCO="on"
CORE_BRANCH="main"
GENERATOR="Unix Makefiles"

Help() {
    echo
    echo "This script builds and installs the last available version of OpenSim-Core in your computer."
    echo "Usage: ./scriptName [OPTION]..."
    echo "Example: ./opensim-core-build.sh -j 4 -d \"Release\""
    echo "    -d         Debug Type. Available Options:"
    echo "                   Release (Default): No debugger symbols. Optimized."
    echo "                   Debug: Debugger symbols. No optimizations (>10x slower). Library names ending with _d."
    echo "                   RelWithDefInfo: Debugger symbols. Optimized. Bigger than Release, but not slower."
    echo "                   MinSizeRel: No debugger symbols. Minimum size. Optimized."
    echo "    -j         Number of jobs to use when building libraries (>=1)."
    echo "    -s         Simple build without moco (Tropter and Casadi disabled)."
    echo "    -c         Branch for opensim-core repository."
    echo "    -n         Use the Ninja generator to build opensim-core. If not set, Unix Makefiles is used."
    echo
    exit
}

# Get flag values if any.
while getopts 'j:d:s:c:n' flag
do
    case "${flag}" in
        j) NUM_JOBS=${OPTARG};;
        d) DEBUG_TYPE=${OPTARG};;
        s) MOCO="off";;
        c) CORE_BRANCH=${OPTARG};;
        n) GENERATOR="Ninja";;
        *) Help;
    esac
done

# Check if parameters are valid.
if [[ $NUM_JOBS -lt 1 ]]
then
    Help;
fi
if [[ $DEBUG_TYPE != "Release" ]] && [[ $DEBUG_TYPE != "Debug" ]] && [[ $DEBUG_TYPE != "RelWithDebInfo" ]] && [[ $DEBUG_TYPE != "MinSizeRel" ]]
then
    Help;
fi

# Show values of flags:
echo
echo "Build script parameters:"
echo "DEBUG_TYPE="$DEBUG_TYPE
echo "NUM_JOBS="$NUM_JOBS
echo "MOCO="$MOCO
echo "CORE_BRANCH="$CORE_BRANCH
echo "GENERATOR="$GENERATOR
echo ""

# Check OS.
echo "LOG: CHECKING OS..."
OS_NAME=$(lsb_release -a)

if [[ $OS_NAME == *"Debian"* ]]; then
   OS_NAME="Debian"
   echo "The OS of this machine is Debian."
elif [[ $OS_NAME == *"Ubuntu"* ]]; then
   OS_NAME="Ubuntu"
   echo "The OS of this machine is Ubuntu."
else
   OS_NAME="Unknown"
   echo "Could not recognize the OS in your machine."
   exit
fi
echo

# Install dependencies from package manager.
echo "LOG: INSTALLING DEPENDENCIES..."
sudo apt-get update && sudo apt-get install --yes build-essential cmake autotools-dev autoconf pkg-config automake libopenblas-dev liblapack-dev freeglut3-dev libxi-dev libxmu-dev doxygen python3 python3-dev python3-numpy python3-setuptools git byacc libssl-dev libpcre3 libpcre3-dev libtool gfortran ninja-build patchelf || ( echo "Installation of dependencies using apt-get failed." && exit )
echo 

# Debian does not have openjdk-8-jdk available, so install from temurin repo.
echo "LOG: INSTALLING JDK 8..."
if [[ $OS_NAME == *"Debian"* ]]; then
   sudo apt-get install -y wget apt-transport-https
   sudo mkdir -p /etc/apt/keyrings || true
   wget -O - https://packages.adoptium.net/artifactory/api/gpg/key/public | sudo tee /etc/apt/keyrings/adoptium.asc
   echo "deb [signed-by=/etc/apt/keyrings/adoptium.asc] https://packages.adoptium.net/artifactory/deb $(awk -F= '/^VERSION_CODENAME/{print$2}' /etc/os-release) main" | sudo tee /etc/apt/sources.list.d/adoptium.list
   sudo apt update # update if you haven't already
   sudo apt install --yes temurin-8-jdk
   # Export JAVA_HOME variable.
   export JAVA_HOME=$(dirname $(dirname $(readlink -f /usr/bin/java)))
elif [[ $OS_NAME == *"Ubuntu"* ]]; then
   sudo apt install --yes openjdk-8-jdk
fi
echo

# Create workspace folder.
mkdir ~/opensim-workspace || true

# Check cmake version installed by apt. If < 3.15, build it from source.
echo "LOG: INSTALLING CMAKE >=3.15..."
cmake_version=$(cmake --version | perl -pe '($_)=/([0-9]+([.][0-9]+))/')
cmake_version_split=(${cmake_version//./ })
min_cmake_version=3.15
min_cmake_version_split=(${min_cmake_version//./ })
cmake_major_less=$(( ${cmake_version_split[0]} < ${min_cmake_version_split[0]} ))
cmake_major_equal=$(( ${cmake_version_split[0]} == ${min_cmake_version_split[0]} ))
cmake_minor_less=$(( ${cmake_version_split[1]} < ${min_cmake_version_split[1]} ))
if [[ $cmake_major_less == 1 ]] || $( [[ $cmake_major_equal == 1 ]] && [[ $cmake_minor_less == 1 ]] ); then
    mkdir ~/opensim-workspace/cmake-3.23.3-source || true
    cd ~/opensim-workspace/cmake-3.23.3-source
    wget -nc -q --show-progress https://github.com/Kitware/CMake/releases/download/v3.23.3/cmake-3.23.3.tar.gz 
    tar -zxvf cmake-3.23.3.tar.gz
    cd ~/opensim-workspace/cmake-3.23.3-source/cmake-3.23.3
    ./bootstrap
    make -j$NUM_JOBS && sudo make install
    source ~/.bashrc && cmake --version
else
    echo "CMake version is higher than 3.15."
fi
echo

# Check python version installed by apt. If < 3.6, build it from source.
echo "LOG: INSTALLING PYTHON >=3.6..."
python_version=$(python3 -c 'import sys; print(str(sys.version_info[0])+"."+str(sys.version_info[1]))')
python_version_split=(${python_version//./ })
min_python_version=3.6
min_python_version_split=(${min_python_version//./ })
python_major_less=$(( ${python_version_split[0]} < ${min_python_version_split[0]} ))
python_major_equal=$(( ${python_version_split[0]} == ${min_python_version_split[0]} ))
python_minor_less=$(( ${python_version_split[1]} < ${min_python_version_split[1]} ))
if [[ $python_major_less == 1 ]] || $( [[ $python_major_equal == 1 ]] && [[ $python_minor_less == 1 ]] ); then
    mkdir ~/opensim-workspace/Python-3.6.9-source || true
    cd ~/opensim-workspace/Python-3.6.9-source
    wget -nc -q --show-progress https://www.python.org/ftp/python/3.6.9/Python-3.6.9.tgz
    tar -xvzf Python-3.6.9.tgz
    cd ~/opensim-workspace/Python-3.6.9-source/Python-3.6.9
    ./configure
    make -j$NUM_JOBS
    sudo make altinstall
    sudo /usr/local/bin/pip3.6 install numpy
else
    echo "Python version is higher than 3.6."
fi
echo

# Download and install SWIG 4.0.2.
echo "LOG: INSTALLING SWIG 4.0.2..."
mkdir -p ~/opensim-workspace/swig-source || true && cd ~/opensim-workspace/swig-source
wget -nc -q --show-progress https://github.com/swig/swig/archive/refs/tags/rel-4.0.2.tar.gz
tar xzf rel-4.0.2.tar.gz && cd swig-rel-4.0.2
sh autogen.sh && ./configure --prefix=$HOME/swig --disable-ccache
make && make -j$NUM_JOBS install  
echo

# Get opensim-core.
echo "LOG: CLONING OPENSIM-CORE..."
git -C ~/opensim-workspace/opensim-core-source pull || git clone https://github.com/opensim-org/opensim-core.git ~/opensim-workspace/opensim-core-source
cd ~/opensim-workspace/opensim-core-source
git checkout $CORE_BRANCH
echo

# Build opensim-core dependencies.
echo "LOG: BUILDING OPENSIM-CORE DEPENDENCIES..."
mkdir -p ~/opensim-workspace/opensim-core-dependencies-build || true
cd ~/opensim-workspace/opensim-core-dependencies-build
cmake ~/opensim-workspace/opensim-core-source/dependencies -DCMAKE_INSTALL_PREFIX=~/opensim-workspace/opensim-core-dependencies-install/ -DSUPERBUILD_ezc3d=on -DOPENSIM_WITH_CASADI=$MOCO -DOPENSIM_WITH_TROPTER=$MOCO
cmake . -LAH
cmake --build . --config $DEBUG_TYPE -j$NUM_JOBS
echo

# Build opensim-core.
echo "LOG: BUILDING OPENSIM-CORE..."
mkdir -p ~/opensim-workspace/opensim-core-build || true
cd ~/opensim-workspace/opensim-core-build
cmake ~/opensim-workspace/opensim-core-source -G"$GENERATOR" -DOPENSIM_DEPENDENCIES_DIR=~/opensim-workspace/opensim-core-dependencies-install/ -DBUILD_JAVA_WRAPPING=on -DBUILD_PYTHON_WRAPPING=on -DOPENSIM_C3D_PARSER=ezc3d -DBUILD_TESTING=off -DCMAKE_INSTALL_PREFIX=~/opensim-core -DOPENSIM_INSTALL_UNIX_FHS=off -DSWIG_DIR=~/swig/share/swig -DSWIG_EXECUTABLE=~/swig/bin/swig
cmake . -LAH
cmake --build . --config $DEBUG_TYPE -j$NUM_JOBS
echo

# Test opensim-core.
echo "LOG: TESTING OPENSIM-CORE..."
cd ~/opensim-workspace/opensim-core-build
# TODO: Temporary for python to find Simbody libraries.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/opensim-workspace/opensim-core-dependencies-install/simbody/lib
ctest --parallel $NUM_JOBS --output-on-failure

# Install opensim-core.
echo "LOG: INSTALL OPENSIM-CORE..."
cd ~/opensim-workspace/opensim-core-build
cmake --install .
echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bashrc
source ~/.bashrc