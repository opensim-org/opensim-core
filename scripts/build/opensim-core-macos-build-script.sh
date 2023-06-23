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
    echo "This script builds and installs the last available version of OpenSim-Gui in your computer."
    echo "Usage: ./scriptName [OPTION]..."
    echo "Example: ./opensim-gui-build.sh -j 4 -d \"Release\""
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

# Check parameters are valid.
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

# Install brew package manager.
echo "LOG: INSTALLING BREW..."

/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" < /dev/null

# Install dependencies from package manager.
echo "LOG: INSTALLING DEPENDENCIES..."
brew install pkgconfig autoconf libtool automake wget pcre doxygen python git ninja cmake
brew reinstall gcc
pip3 install cython
pip3 install numpy
echo

# Install Java 8 from temurin repositories.
brew tap homebrew/cask-versions
brew install --cask temurin8
export JAVA_HOME=$(/usr/libexec/java_home -v 1.8)
echo $JAVA_HOME
 
# Create workspace folder.
mkdir ~/opensim-workspace || true

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
cmake ~/opensim-workspace/opensim-core-source  -G"$GENERATOR" -DOPENSIM_DEPENDENCIES_DIR=~/opensim-workspace/opensim-core-dependencies-install/ -DBUILD_JAVA_WRAPPING=on -DBUILD_PYTHON_WRAPPING=on -DOPENSIM_C3D_PARSER=ezc3d -DBUILD_TESTING=off -DCMAKE_INSTALL_PREFIX=~/opensim-core -DOPENSIM_INSTALL_UNIX_FHS=off -DSWIG_DIR=~/swig/share/swig -DSWIG_EXECUTABLE=~/swig/bin/swig -DJAVA_HOME=$(/usr/libexec/java_home -v 1.8) -DJAVA_INCLUDE_PATH=$(/usr/libexec/java_home -v 1.8)/include -DJAVA_INCLUDE_PATH2=$(/usr/libexec/java_home -v 1.8)/include/darwin -DJAVA_AWT_INCLUDE_PATH=$(/usr/libexec/java_home -v 1.8)/include
cmake . -LAH
cmake --build . --config $DEBUG_TYPE -j$NUM_JOBS
echo

# Test opensim-core.
echo "LOG: Testing opensim-core."
cd ~/opensim-workspace/opensim-core-build
python -m unittest discover --start-directory opensim/tests --verbose
echo

# Install opens-core.
echo "LOG: INSTALL OPENSIM-CORE."
cd ~/opensim-workspace/opensim-core-build
cmake --install .
echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bashrc
source ~/.bashrc