$PROJECT = $args[0]
$SOURCE_DIR = $args[1]
$BUILD_DIR = $args[2]

$CURR_DIR = (Get-Location)

if(-not (Test-Path "$BUILD_DIR")) {
  New-Item -Path $BUILD_DIR -ItemType Directory -Force | Out-Null
}

if(! $USE_CACHE) {
  Write-Host "---- Not downloading cache. Caching disabled."
  return
}

if($env:APPVEYOR_REPO_BRANCH -eq "master") {
  Write-Host "---- Not downloading cache. This is master build."
  return
}

if($env:APPVEYOR_REPO_COMMIT_MESSAGE -match "\[make clean\]") { 
  Write-Host "---- Not downloading cache. Make clean." 
  return
}

Write-Host '---- Checking for availability of cached build directory on Bintray.'
if($env:CMAKE_GENERATOR -like "*Win64") {
  $COMPILER = "msvc_win64"  
} else {
  $COMPILER = "msvc_win32"
}
$PACKAGE_NAME = $env:Platform + "_" + $COMPILER + "_" + "Release"

Set-Location $env:OPENSIM_SOURCE_DIR
$BRANCHTIP = $env:APPVEYOR_REPO_COMMIT
git fetch --quiet origin master:master
$BRANCHBASE = (git merge-base master $BRANCHTIP)

# Set the timestamps of all files back.
$timestamp = Get-Date -Year 1990 -Month 01 -Day 01 -Hour 01 -Minute 01 -Second 01
Get-ChildItem -Recurse | ForEach-Object { $_.LastWriteTime = $timestamp }

# Touch the files that this branch has modified after its birth.
$CHANGED_FILES = (git diff --name-only $BRANCHBASE build_cache_dev_branch)
ForEach($FILE in $CHANGED_FILES) {
  (Get-Item $FILE).LastWriteTime = Get-Date 
}

$BUILD_DIRNAME = (get-item $BUILD_DIR).name
$ZIP = $BUILD_DIRNAME + ".zip"
$LETTERS = 'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'
$URL = "https://dl.bintray.com/opensim/${PROJECT}/${PACKAGENAME}/${BRANCHBASE}"
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
