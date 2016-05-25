set -e

# Arguments
PROJECT=$1
SOURCE_DIR=$2

CURR_DIR=$(pwd)
# Turn relative paths into absolute paths.
if [ "${SOURCE_DIR:0:1}" != "/" ]; then
  SOURCE_DIR=${CURR_DIR}/${SOURCE_DIR}
fi

if ! $USE_CACHE; then
  echo "---- Caching disabled."
  cd $CURR_DIR
  return
fi

echo "---- Retrieving list of versions on cache."
if [[ "$CC" == *gcc* ]]; then export COMPILER=gcc; fi
if [[ "$CC" == *clang* ]]; then export COMPILER=clang; fi
PACKAGENAME="${MACHTYPE}_${COMPILER}_${BTYPE}"
URL="https://api.bintray.com/packages/opensim/${PROJECT}/${PACKAGENAME}"
if [ "$TRAVIS_OS_NAME" == "osx" ]; then brew update; brew install jq; fi
CACHED_VERSIONS=$(curl --silent -u$BINTRAY_CREDS $URL | jq .versions[] | sed 's/"//g')

echo $CACHED_VERSIONS
echo "--- Retrieving list of used versions."
cd $SOURCE_DIR
git fetch -q origin master:master
BRANCHES=$(git ls-remote --heads origin | sed 's/.*refs\/heads\///')
echo $BRANCHES
for b in $BRANCHES; do 
  git fetch -q origin $b:$b 
  BRANCHBASE=$(git merge-base master $b)
  CACHED_VERSIONS="${CACHED_VERSIONS/$BRANCHBASE/}"
done
echo $CACHED_VERSIONS

cd $CURR_DIR
