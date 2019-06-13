bash .github/travis_common.sh 

# Run superbuild to download, configure, build and install dependencies.
mkdir $TRAVIS_BUILD_DIR/../opensim-moco_dependencies_build
cd $TRAVIS_BUILD_DIR/../opensim-moco_dependencies_build 
DEP_CMAKE_ARGS=($TRAVIS_BUILD_DIR/dependencies)
DEP_CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE=$BTYPE)
DEP_CMAKE_ARGS+=(-DOPENSIM_JAVA_WRAPPING=on -DOPENSIM_PYTHON_WRAPPING=on -DSWIG_EXECUTABLE=$HOME/swig/bin/swig)
if [ "$TRAVIS_OS_NAME" = "osx" ]; then DEP_CMAKE_ARGS+=(-DCMAKE_OSX_DEPLOYMENT_TARGET=$OSX_TARGET); fi

printf '%s\n' "${DEP_CMAKE_ARGS[@]}"
cmake "${DEP_CMAKE_ARGS[@]}"
ls ~
# make -j$NPROC opensim-core
make -j$NPROC colpack
# make -j$NPROC adolc
# make -j$NPROC ipopt
# make -j$NPROC casadi
# make -j$NPROC

# Zip up the installation using a file name that identifies where
# the binaries were built.
mkdir ~/to_deploy
ZIPNAME=opensim-moco-deps.zip
# Zip up Moco relative to where it's installed.
ls $TRAVIS_BUILD_DIR
ls $TRAVIS_BUILD_DIR/..
ls $TRAVIS_BUILD_DIR/../moco_dependencies_install
cd $TRAVIS_BUILD_DIR/../moco_dependencies_install
# Leave symlinks intact.
zip --symlinks --recurse-paths --quiet ~/to_deploy/$ZIPNAME .

