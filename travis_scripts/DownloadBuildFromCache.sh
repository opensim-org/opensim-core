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

if [ ! -d $BUILD_DIR ]; then mkdir $BUILD_DIR; fi
if ! $USE_CACHE; then
  echo "---- Not downloading cache. Caching disabled."
  cd $CURR_DIR
  return
fi
if [ "$TRAVIS_PULL_REQUEST" == "false" ] && [ "$TRAVIS_BRANCH" == "master" ]; then 
  echo "---- Not downloading cache. This is master build."
  cd $CURR_DIR
  return
fi
cd $SOURCE_DIR
if $(git log -n1 --format="%B" | grep --quiet '\[ci make clean\]'); then
  echo "---- Not downloading cache. Make clean."
  cd $CURR_DIR
  return
fi

echo '---- Checking for availability of cached build directory on Bintray.'
if [[ "$CC" == *gcc* ]]; then export COMPILER=gcc; fi
if [[ "$CC" == *clang* ]]; then export COMPILER=clang; fi
PACKAGENAME="${MACHTYPE}_${COMPILER}_${BTYPE}"

BRANCHTIP=$(git log -n1 --format='%H')
git fetch --quiet --unshallow
git fetch --quiet origin master:master
BRANCHBASE=$(git merge-base master ${BRANCHTIP})
# Set timestamp of all files back.
find . -name '*' | while read f; do touch -m -t"199001010101" $f; done
# Touch the files that this branch has modified after its birth.
git diff --name-only $BRANCHBASE $BRANCHTIP | while read f; do touch $f; done

BUILD_DIRNAME=$(basename $BUILD_DIR)
TARBALL=${BUILD_DIRNAME}.tar.gz
LETTERS='a b c d e f g h i j k l m n o p q r s t u v w x y z'
URL="https://dl.bintray.com/opensim/${PROJECT}/${PACKAGENAME}/${BRANCHBASE}"
echo "---- Looking for opensim/${PROJECT}/${PACKAGENAME}/${BRANCHBASE}"
cd ${BUILD_DIR}/..
for i in $LETTERS; do 
  piece=${TARBALL}a$i 
  curl -s -L $URL/$piece -o $piece
  if [ $(wc -c < $piece) -lt 100 ]; then 
    rm $piece 
    break 
  else 
    echo "---- Downloaded piece $piece"
  fi 
done
if [ ! -f ${TARBALL}aa ]; then 
  echo '---- Cache not found.'
  return
fi
echo '---- Joining the pieces downloaded.'
cat ${TARBALL}* > ${TARBALL}
echo '---- Decompressing tarball.'
tar -xzf ${TARBALL}
echo '---- Cleaning up.'
rm -f ${TARBALL}*
cd $CURR_DIR
