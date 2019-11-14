# Terminate on error, print every line
set -ev

bash .github/travis_common.sh

# Download dependencies
wget https://sourceforge.net/projects/myosin/files/opensim-moco/opensim-moco-dep-opensim-core.zip/download -O ~/opensim-moco-dep-opensim-core.zip
unzip ~/opensim-moco-dep-opensim-core.zip -d $TRAVIS_BUILD_DIR/../

wget https://sourceforge.net/projects/myosin/files/opensim-moco/opensim-moco-dep-adolc.zip/download -O ~/opensim-moco-dep-adolc.zip
unzip ~/opensim-moco-dep-adolc.zip -d $TRAVIS_BUILD_DIR/../

wget https://sourceforge.net/projects/myosin/files/opensim-moco/opensim-moco-dep-casadi.zip/download -O ~/opensim-moco-dep-casadi.zip
unzip ~/opensim-moco-dep-casadi.zip -d $TRAVIS_BUILD_DIR/../

wget https://sourceforge.net/projects/myosin/files/opensim-moco/opensim-moco-dep-eigen.zip/download -O ~/opensim-moco-dep-eigen.zip
unzip ~/opensim-moco-dep-eigen.zip -d $TRAVIS_BUILD_DIR/../

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
OSIM_CMAKE_ARGS+=(-DMOCO_JAVA_BINDINGS=$WRAP -DMOCO_PYTHON_BINDINGS=$WRAP)
  
# Bindings.
OSIM_CMAKE_ARGS+=(-DSWIG_EXECUTABLE=$HOME/swig/bin/swig)

# CMake wasn't finding these libraries for some reason, so we hardcode their
# locations.
OSIM_CMAKE_ARGS+=(-Dpkgcfg_lib_IPOPT_m=/usr/lib/libm.dylib)
OSIM_CMAKE_ARGS+=(-Dpkgcfg_lib_IPOPT_dl=/usr/lib/libdl.dylib)
OSIM_CMAKE_ARGS+=(-Dpkgcfg_lib_IPOPT_System=/System/Library/Frameworks/System.framework)
  
# Doxygen.
if [[ "$DOXY" = "on" && "$TRAVIS_OS_NAME" = "linux" ]]; then OSIM_CMAKE_ARGS+=(-DDOXYGEN_EXECUTABLE=$HOME/doxygen/doxygen-1.8.10/bin/doxygen); fi

  
printf '%s\n' "${OSIM_CMAKE_ARGS[@]}"
cmake "${OSIM_CMAKE_ARGS[@]}"

cmake . -LAH

VERSION=`cmake -L . | grep MOCO_FULL_VERSION | cut -d "=" -f2`
echo $VERSION

make -j$NPROC;

ctest -j8 --output-on-failure $CTEST_FLAGS --exclude-regex $TESTS_TO_EXCLUDE

## Build doxygen documentation.
if [ "$DOXY" = "on" ]; then make Moco_doxygen; fi
  
## Install. Suppress output.
make -j8 install > /dev/null

# Zip up the installation using a file name that identifies where
# the binaries were built.
mkdir ~/to_deploy
# Zip up Moco relative to where it's installed.
cd ~
OS_NAME=$TRAVIS_OS_NAME
if [ "$TRAVIS_OS_NAME" = "osx" ]; then
    OS_NAME=mac
fi
ZIPNAME=opensim-moco-${VERSION}-${OS_NAME}
if [ "$TRAVIS_BRANCH" != "master" ]; then
    if [ "$TRAVIS_BRANCH" != "$TRAVIS_TAG" ]; then
        # If there is a tag, then TRAVIS_BRANCH is equal to TRAVIS_TAG,
        # and the version is already the tag, so we don't need to
        # repeat the tag.
        ZIPNAME=${ZIPNAME}-${TRAVIS_BRANCH}
    fi
fi
if [ ! -z "$TRAVIS_PULL_REQUEST_BRANCH" ]; then
    ZIPNAME=${ZIPNAME}-${TRAVIS_PULL_REQUEST_BRANCH}
fi
ZIPNAME=${ZIPNAME}.zip

if [ ! -z "$TRAVIS_TAG" ]; then

    # Leave symlinks intact.
    mv opensim-moco opensim-moco-${VERSION}
    zip --symlinks --recurse-paths --quiet ~/to_deploy/$ZIPNAME opensim-moco-${VERSION}

    ## Set up ssh for sourceforge.
    cd $TRAVIS_BUILD_DIR
    if [[ "$DEPLOY" = "yes" ]]; then PREP_SOURCEFORGE_SSH=0; else PREP_SOURCEFORGE_SSH=1; fi
    # Decrypt the private key stored in the repository to the tmp dir.
    if [ $PREP_SOURCEFORGE_SSH = "0" ]; then openssl aes-256-cbc -K $encrypted_3d3280d08c79_key -iv $encrypted_3d3280d08c79_iv -in .github/.deploy_myosin_sourceforge_rsa.enc -out /tmp/deploy_myosin_sourceforge_rsa -d; fi
    # Start the ssh agent.
    if [ $PREP_SOURCEFORGE_SSH = "0" ]; then eval "$(ssh-agent -s)"; fi
    # Register this private key with this client (the travis machine).
    if [ $PREP_SOURCEFORGE_SSH = "0" ]; then chmod 600 /tmp/deploy_myosin_sourceforge_rsa; fi
    if [ $PREP_SOURCEFORGE_SSH = "0" ]; then ssh-add /tmp/deploy_myosin_sourceforge_rsa; fi

    # Uploads to sourceforge.net/projects/myosin
    # See https://docs.travis-ci.com/user/deployment/custom/
    # '--archive' preserves symlinks.
    rsync --archive --compress --verbose ~/to_deploy/$ZIPNAME opensim-bot@frs.sourceforge.net:/home/frs/project/myosin/opensim-moco/

fi
