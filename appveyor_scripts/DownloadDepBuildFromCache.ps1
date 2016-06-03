$PROJECT = $args[0]
$SOURCE_DIR = $args[1]
$BUILD_DIR = $args[2]

$CURR_DIR = (Get-Location)

if(! $USE_CACHE) {
  Write-Host "---- Not uploading cache. Caching disabled."
  return
}

Set-Location $BUILD_DIR
# Extract the commit id of the dependency.
$BRANCHTIP = (Select-String -Pattern 'COMMAND .*git.*checkout ([0-9a-z]{40})' -Path .\tmp\*gitclone.cmake).Matches.Groups[1].Value
$SOURCEURL = (Select-String -Pattern 'COMMAND .*git.*clone .*("https://.*git")' -Path .\tmp\*gitclone.cmake).Matches.Groups[1].Value
Set-Location $SOURCE_DIR/..
git clone --quiet $SOURCEURL (Get-Item $SOURCE_DIR).Name
Set-Location $SOURCE_DIR
git checkout --quiet $BRANCHTIP

Write-Host '---- Checking for availability of cached build directory on Bintray.'
if($env:CMAKE_GENERATOR -like "*Win64") {
  $COMPILER = "msvc_win64"  
} else {
  $COMPILER = "msvc_win32"
}
$PACKAGENAME = $env:Platform + "_" + $COMPILER + "_" + "Release"

# Set the timestamps of all files back.
$timestamp = Get-Date -Year 1990 -Month 01 -Day 01 -Hour 01 -Minute 01 -Second 01
Get-ChildItem -Recurse | ForEach-Object { $_.LastWriteTime = $timestamp }

$ZIP = (Get-Item $BUILD_DIR).Name + ".zip"
$LETTERS = 'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'
$URL = "https://dl.bintray.com/opensim/${PROJECT}/${PACKAGENAME}/${BRANCHTIP}"
Write-Host "---- Looking for opensim/${PROJECT}/${PACKAGENAME}/${BRANCHTIP}"
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
  Remove-Item $ZIP + "_$LETTER"
}
$FILESTREAM.close()

Write-Host "---- Decompressing zip."
Add-Type -AssemblyName "System.IO.Compression.FileSystem"
Write-Host $ZIP
Write-Host (Get-Location).Path
[System.IO.Compression.ZipFile]::ExtractToDirectory("$ZIP", (Get-Location).Path)

Write-Host "---- Cleaning up."
Remove-Item $ZIP
Set-Location $CURR_DIR
