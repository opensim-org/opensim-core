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

if ! $USE_CACHE; then
  echo "---- Not uploading cache. Caching disabled."
  cd $CURR_DIR
  return
fi

cd $SOURCE_DIR
if [ "$PROJECT" == "opensim-core" ]; then
  if [ "$TRAVIS_PULL_REQUEST" != "false" ] || [ "$TRAVIS_BRANCH" != "master" ]; then
      echo "---- Not caching build directory. This is not master build."
      cd $CURR_DIR
      return
  fi
fi

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
  curl --upload-file $piece -u$BINTRAY_CREDS ${URL}/${piece}
  echo 
done
URL="https://api.bintray.com/content/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}/publish"
echo '---- Publishing uploaded build directory.'
curl --request POST -u$BINTRAY_CREDS $URL
echo
echo '---- Cleaning up.'
rm ${TARBALL}*
cd $CURR_DIR
