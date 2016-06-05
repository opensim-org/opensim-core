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
$PACKAGENAME = $env:Platform + "_" + $COMPILER + "_" + "Release"

Set-Location $env:OPENSIM_SOURCE_DIR
$BRANCHTIP = $env:APPVEYOR_REPO_COMMIT
git fetch --quiet origin master:master
$BRANCHBASE = (git merge-base master $BRANCHTIP)

# Set the timestamps of all files back.
$timestamp = Get-Date -Year 1990 -Month 01 -Day 01 -Hour 01 -Minute 01 -Second 01
Get-ChildItem -Recurse | ForEach-Object { $_.LastWriteTime = $timestamp }

# Touch the files that this branch has modified after its birth.
$CHANGED_FILES = (git diff --name-only $BRANCHBASE $BRANCHTIP)
ForEach($FILE in $CHANGED_FILES) {
  (Get-Item $FILE).LastWriteTime = Get-Date 
}

$BUILD_DIRNAME = (get-item $BUILD_DIR).name
$ZIP = $BUILD_DIRNAME + ".zip"
$LETTERS = 'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'
$URL = "https://dl.bintray.com/opensim/${PROJECT}/${PACKAGENAME}/${BRANCHBASE}"
Write-Host "---- Looking for opensim/${PROJECT}/${PACKAGENAME}/${BRANCHBASE}"
Set-Location ${BUILD_DIR}/..
ForEach($LETTER in $LETTERS) {
  $PIECE = $ZIP + "_" + $LETTER
  try {
    Invoke-WebRequest $URL/$PIECE -OutFile $PIECE
  } catch {
    break
  }
  Write-Host "---- Downloaded piece $PIECE."
}

if(-not (Test-Path "${ZIP}_a")) {
  Write-Host "---- Cache not found."
  Set-Location $CURR_DIR
  return
}

Write-Host '---- Joining the pieces downloaded.'
$ZIP = (Get-Location).Path + "/" + $ZIP
$FILESTREAM = [System.IO.File]::OpenWrite($ZIP)
$BUFFER = New-Object byte[] 200mb
ForEach($LETTER in $LETTERS) {
  try {
    $PIECE = [System.IO.File]::OpenRead($ZIP + "_$LETTER")
  } catch {
    break
  }
  $BYTESREAD = $PIECE.Read($BUFFER, 0, $BUFFER.Length)
  $FILESTREAM.Write($BUFFER, 0, $BYTESREAD)
  $PIECE.Close()
  Remove-Item "${ZIP}_${LETTER}"
}
$FILESTREAM.close()

Write-Host "---- Decompressing zip."
choco install --yes unzip > $null
Remove-Item -Recurse $BUILD_DIR
unzip -q "$ZIP"

Write-Host "---- Cleaning up."
Remove-Item $ZIP
Set-Location $CURR_DIR
