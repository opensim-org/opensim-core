$PROJECT = $args[0]
$SOURCE_DIR = $args[1]

$CURR_DIR = (Get-Location)

if(! $USE_CACHE) {
  Write-Host "---- Caching disabled."
  return
}

if($env:APPVEYOR_REPO_BRANCH -ne "master") {
  Write-Host "---- Skipping cache cleanup. This is not master build."
  return
}

Write-Host "---- Retrieving list of versions on cache."
if($env:CMAKE_GENERATOR -like "*Win64") {
  $COMPILER = "msvc_win64"  
} else {
  $COMPILER = "msvc_win32"
}
$PACKAGENAME = $env:Platform + "_" + $COMPILER + "_" + "Release"
$URL = "https://api.bintray.com/packages/opensim/${PROJECT}/${PACKAGENAME}"
$CACHED_VERSIONS = (curl --silent -u$BINTRAY_CREDS $URL | jq .versions[] | sed 's/"//g')

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
