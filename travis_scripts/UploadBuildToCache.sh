set -e
# Arguments
PROJECT=$1
SOURCE_DIR=$2
BUILD_DIR=$3

CURR_DIR=$(pwd)
# Turn relative paths into absolute paths.
if [ "${SOURCE_DIR:0:1}" != "/" ]; then
  SOURCE_DIR=${CURR_DIR}/${SOURCE_DIR}
fi
if [ "${BUILD_DIR:0:1}" != "/" ]; then
  BUILD_DIR=${CURR_DIR}/${BUILD_DIR}
fi

cd $SOURCE_DIR
if [ "$PROJECT" == "opensim-core" ]; then
  # Make sure the branch is master.
  CURRBRANCH=$(git reflog | tail -n2 | head -n1 | sed 's/.*checkout: moving from \([^ ]*\) to.*/\1/')
  if [ "$CURRBRANCH" != "master" ]; then
    # Cache branch base in master if it does not exist and if the user instructs so.
    if $(git log -n1 --format="%B" | grep --quiet '[cache master]'); then
      BRANCHTIP=$(git log -n1 --format='%H')
      git fetch --quiet origin master:master
      BRANCHBASE=$(git merge-base master ${BRANCHTIP})
      git checkout --quiet $BRANCHBASE
      cd $BUILD_DIR
      make -j$NPROC
    else 
      echo "---- Not caching build directory. Current branch (${CURRBRANCH}) is not master."
      cd $CURR_DIR
      return
    fi
  fi
fi

cd $SOURCE_DIR
MASTERTIP=$(git log -n1 --format="%H")
if [[ "$CC" == *gcc* ]]; then export COMPILER=gcc; fi
if [[ "$CC" == *clang* ]]; then export COMPILER=clang; fi
PACKAGENAME="${MACHTYPE}_${COMPILER}_${BTYPE}"
URL="https://dl.bintray.com/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}"
BUILD_DIRNAME=$(basename $BUILD_DIR)
TARBALL=${BUILD_DIRNAME}.tar.gz
if curl --head --fail --silent --location ${URL}/${TARBALL}aa -o /dev/null; then
  echo "---- Already cached: $PROJECT."
  return
fi

cd ${BUILD_DIR}/..
echo '---- Compressing build directory into a tarball.'
tar -czf $TARBALL $BUILD_DIRNAME
echo '---- Splitting tarball into smaller pieces for upload.'
split -b 200m $TARBALL $TARBALL
rm $TARBALL
URL="https://api.bintray.com/content/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}/${PACKAGENAME}/${MASTERTIP}"
PIECES=$(ls ${TARBALL}a*)
for piece in $PIECES; do 
  echo "---- Uploading piece ${piece} to opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}"
  curl --upload-file $piece -uklshrinidhi:440061321dba00a68210b482261154ea58d03f00 ${URL}/${piece}
  echo 
done
URL="https://api.bintray.com/content/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}/publish"
echo '---- Publishing uploaded build directory.'
curl --request POST -uklshrinidhi:440061321dba00a68210b482261154ea58d03f00 $URL
echo
echo '---- Cleaning up.'
rm ${TARBALL}*
cd $CURR_DIR

