$CURR_DIR = (pwd)

if(! $USE_CACHE) {
  Write-Host "---- Not uploading cache. Caching disabled."
  return
}

#if($env:APPVEYOR_REPO_BRANCH -neq "master") {
#  Write-Host "---- Not uploading cache. This is not master build."
#  return
#}
Set-Location $env:OPENSIM_SOURCE_DIR
git fetch origin master:master
git checkout master
Set-Location $env:OPENSIM_BUILD_DIR
cmake --build . --config Release -- /maxcpucount:4 /verbosity:quiet

$MASTERTIP = (git log -n1 --format="%H")
# $MASTERTIP = $env:APPVEYOR_REPO_COMMIT
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
    $PIECE = [System.IO.File]::OpenWrite((Get-Item $ZIP).FullName + "_$_")
    $PIECE.Write($BUFFER, 0, $BYTESREAD)
    $PIECE.Close()
  }
})
Remove-Item $ZIP

$PASSWORD = ConvertTo-SecureString "440061321dba00a68210b482261154ea58d03f00" -AsPlainText -Force
$CREDS = New-Object System.Management.Automation.PSCredential("klshrinidhi", $PASSWORD)
$URL = "https://api.bintray.com/content/opensim/opensim-core/${PACKAGENAME}/${MASTERTIP}/${PACKAGENAME}/${MASTERTIP}"
(Get-Item "${ZIP}_*").ForEach({
  Write-Host "---- Uploading piece $_ to opensim/opensim-core/${PACKAGENAME}/${MASTERTIP}"
  Invoke-WebRequest -Credential $CREDS -Method PUT -InFile $_ $URL/$_ | Out-Null
})

$URL = "https://api.bintray.com/content/opensim/opensim-core/${PACKAGENAME}/${MASTERTIP}/publish"
Write-Host '---- Publishing uploaded build directory.'
Invoke-WebRequest -Credential $CREDS -Method POST $URL | Out-Null
Write-Host '---- Cleaning up.'
Remove-Item ${ZIP}*
Set-Location $CURR_DIR
