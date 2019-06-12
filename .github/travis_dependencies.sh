bash .github/travis_common.sh 

# Run superbuild to download, configure, build and install dependencies.
mkdir ~/opensim-moco_dependencies_build
cd ~/opensim-moco_dependencies_build 
DEP_CMAKE_ARGS=($TRAVIS_BUILD_DIR/dependencies)
DEP_CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE=$BTYPE)
DEP_CMAKE_ARGS+=(-DOPENSIM_JAVA_WRAPPING=on -DOPENSIM_PYTHON_WRAPPING=on -DSWIG_EXECUTABLE=$HOME/swig/bin/swig)
if [ "$TRAVIS_OS_NAME" = "osx" ]; then DEP_CMAKE_ARGS+=(-DCMAKE_OSX_DEPLOYMENT_TARGET=$OSX_TARGET); fi

printf '%s\n' "${DEP_CMAKE_ARGS[@]}"
cmake "${DEP_CMAKE_ARGS[@]}"
make -j$NPROC opensim-core
make -j$NPROC adolc
make -j$NPROC ipopt
make -j$NPROC casadi
make -j$NPROC

# Zip up the installation using a file name that identifies where
# the binaries were built.
mkdir ~/to_deploy
ZIPNAME=opensim-moco-latest_${TRAVIS_OS_NAME}_${BTYPE}.zip
# Zip up opensim relative to where it's installed.
cd $TRAVIS_BUILD_DIR/
# Leave symlinks intact.
zip --symlinks --recurse-paths --quiet ~/to_deploy/$ZIPNAME ~/moco_dependencies_install

