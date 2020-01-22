# Terminate on error, print every line
set -ev

bash .github/travis_common.sh

# Run superbuild to download, configure, build and install dependencies.
mkdir $TRAVIS_BUILD_DIR/../opensim-moco_dependencies_build
cd $TRAVIS_BUILD_DIR/../opensim-moco_dependencies_build 
DEP_CMAKE_ARGS=($TRAVIS_BUILD_DIR/dependencies)
DEP_CMAKE_ARGS+=(-DCMAKE_BUILD_TYPE=$BTYPE)
DEP_CMAKE_ARGS+=(-DOPENSIM_JAVA_WRAPPING=on -DOPENSIM_PYTHON_WRAPPING=on -DOPENSIM_PYTHON_VERSION=3 -DSWIG_EXECUTABLE=$HOME/swig/bin/swig)
if [ "$TRAVIS_OS_NAME" = "osx" ]; then DEP_CMAKE_ARGS+=(-DCMAKE_OSX_DEPLOYMENT_TARGET=$OSX_TARGET); fi

printf '%s\n' "${DEP_CMAKE_ARGS[@]}"
cmake "${DEP_CMAKE_ARGS[@]}"
ls ~
if [ $1 = "casadi" ]; then
    make -j$NPROC ipopt
fi
make -j$NPROC $1

# Zip up the installation using a file name that identifies where
# the binaries were built.
mkdir ~/to_deploy
ZIPNAME=opensim-moco-dep-$1.zip
# Zip up Moco relative to where it's installed.
ls $TRAVIS_BUILD_DIR
ls $TRAVIS_BUILD_DIR/..
ls $TRAVIS_BUILD_DIR/../moco_dependencies_install
cd $TRAVIS_BUILD_DIR/..
# Leave symlinks intact.
zip --symlinks --recurse-paths --quiet ~/to_deploy/$ZIPNAME moco_dependencies_install

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

rsync --archive --compress --verbose ~/to_deploy/ opensim-bot@frs.sourceforge.net:/home/frs/project/myosin/opensim-moco/
