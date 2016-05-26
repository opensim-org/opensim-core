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

if [ "$TRAVIS_PULL_REQUEST" != "false" ] || [ "$TRAVIS_BRANCH" != "master" ]; then 
  echo "---- Skipping cache cleanup. This is not master build."
  cd $CURR_DIR
  return
fi

if [ "$TRAVIS_OS_NAME" == "osx" ]; then brew update; brew install jq; fi

echo "---- Retrieving list of versions on cache."
if [[ "$CC" == *gcc* ]]; then export COMPILER=gcc; fi
if [[ "$CC" == *clang* ]]; then export COMPILER=clang; fi
PACKAGENAME="${MACHTYPE}_${COMPILER}_${BTYPE}"
URL="https://api.bintray.com/packages/opensim/${PROJECT}/${PACKAGENAME}"
CACHED_VERSIONS=$(curl --silent -u$BINTRAY_CREDS $URL | jq .versions[] | sed 's/"//g')

echo "---- Retrieving list of currently used versions."
cd $SOURCE_DIR
git fetch -q origin master:master
# Following line includes master. This is to make sure we keep cache for latest commit
# on master.
BRANCHES=$(git ls-remote --heads origin | sed 's/.*refs\/heads\///')
for b in $BRANCHES; do 
  git fetch -q origin $b:$b 
  BRANCHBASE=$(git merge-base master $b)
  CACHED_VERSIONS="${CACHED_VERSIONS/$BRANCHBASE/}"
done

URL="$URL/versions"
for v in $CACHED_VERSIONS; do
  echo "---- Deleting cache for version $v."
  curl --silent -X DELETE -u$BINTRAY_CREDS ${URL}/${v}
  echo
done
cd $CURR_DIR
