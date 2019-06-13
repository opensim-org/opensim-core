bash .github/travis_common.sh  

# Download dependencies
wget https://sourceforge.net/projects/myosin/files/opensim-moco/opensim-moco-deps.zip/download ~/opensim-moco-deps.zip
unzip ~/opensim-moco-deps.zip -d $TRAVIS_BUILD_DIR/../
ls $TRAVIS_BUILD_DIR/..
ls $TRAVIS_BUILD_DIR/../moco_dependencies_install

mkdir $TRAVIS_BUILD_DIR/../opensim-moco-build && cd $TRAVIS_BUILD_DIR/../opensim-moco-build
## Store CMake arguments in bash array.
# https://stackoverflow.com/questions/1951506/add-a-new-element-to-an-array-without-specifying-the-index-in-bash
OSIM_CMAKE_ARGS=($TRAVIS_BUILD_DIR -DCMAKE_BUILD_TYPE=$BTYPE)

OSIM_CMAKE_ARGS+=(-DCMAKE_INSTALL_PREFIX=~/opensim-moco)

# The minimum macOS/OSX version we support.
if [ "$TRAVIS_OS_NAME" = "osx" ]; then OSIM_CMAKE_ARGS+=(-DCMAKE_OSX_DEPLOYMENT_TARGET=$OSX_TARGET); fi
  
# Dependencies.
OSIM_CMAKE_ARGS+=(-DMOCO_JAVA_BINDINGS=on -DMOCO_PYTHON_BINDINGS=on)
  
# Bindings.
OSIM_CMAKE_ARGS+=(-DBUILD_PYTHON_WRAPPING=$WRAP -DBUILD_JAVA_WRAPPING=$WRAP -DSWIG_EXECUTABLE=$HOME/swig/bin/swig)
# On Mac, use system python instead of Homebrew python.
if [ "$TRAVIS_OS_NAME" = "osx" ]; then OSIM_CMAKE_ARGS+=(-DPYTHON_EXECUTABLE=/usr/bin/python); fi
  
# Doxygen.
if [[ "$DOXY" = "on" && "$TRAVIS_OS_NAME" = "linux" ]]; then OSIM_CMAKE_ARGS+=(-DDOXYGEN_EXECUTABLE=$HOME/doxygen/doxygen-1.8.10/bin/doxygen); fi
  
printf '%s\n' "${OSIM_CMAKE_ARGS[@]}"
cmake "${OSIM_CMAKE_ARGS[@]}"
  
make -j$NPROC;

script:
ctest -j8 --output-on-failure $CTEST_FLAGS --exclude-regex $TESTS_TO_EXCLUDE

## Build doxygen documentation.
if [ "$DOXY" = "on" ]; then make doxygen; fi
  
## Install. Suppress output.
make -j8 install > /dev/null

# Zip up the installation using a file name that identifies where
# the binaries were built.
mkdir ~/to_deploy
# Zip up Moco relative to where it's installed.
cd ~
# Leave symlinks intact.
zip --symlinks --recurse-paths --quiet ~/to_deploy/$ZIPNAME opensim-moco
