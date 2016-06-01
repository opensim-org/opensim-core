$CURR_DIR = (pwd)

if(! $USE_CACHE) {
  Write-Host "---- Not uploading cache. Caching disabled."
  return
}

if($env:APPVEYOR_REPO_BRANCH -neq "master") {
  Write-Host "---- Not uploading cache. This is not master build."
  return
}

$MASTERTIP = $env:APPVEYOR_REPO_COMMIT
if($env:CMAKE_GENERATOR -like "*Win64") {
  $COMPILER = "msvc_win64"  
} else {
  $COMPILER = "msvc_win32"
}
$PACKAGE_NAME = $env:Platform + "_" + $COMPILER + "_" + "Release"
$URL="https://dl.bintray.com/opensim/opensim-core/${PACKAGENAME}/${MASTERTIP}"
$BUILD_DIRNAME = (get-item $env:OPENSIM_BUILD_DIR).name
$ZIP = $BUILD_DIRNAME + ".zip"
# If already cached, return.
try {
  if ( (Invoke-WebRequest -Method Head "${URL}/${ZIP}_a").StatusCode -eq 200 ) {
    Write-Host "---- Already cached: opensim-core."
    return
  }
} catch {}

Set-Location $env:OPENSIM_BUILD_DIR/..
Write-Host '---- Compressing build directory into a zip.'
Compress-Archive -Path $BUILD_DIRNAME -DestinationPath $ZIP
Write-Host '---- Splitting zip into smaller pieces for upload.'
$FILESTREAM = [System.IO.File]::OpenRead((Get-Item $ZIP))
$BUFFER = New-Object byte[] 200mb
$LETTERS = 'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'
$LETTERS.ForEach({
  $BYTESREAD = $FILESTREAM.Read($BUFFER, 0, $BUFFER.Length)
  if($BYTESREAD -eq 0) {
    break
  } else {
    $PIECE = [System.IO.File]::OpenWrite("./" + $ZIP + "_$_")
    $PIECE.Write($BUFFER, 0, $BYTESREAD)
    $PIECE.Close()
  }
})
Remove-Item $ZIP

$URL = "https://api.bintray.com/content/opensim/opensim-core/${PACKAGENAME}/${MASTERTIP}/${PACKAGENAME}/${MASTERTIP}"
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
