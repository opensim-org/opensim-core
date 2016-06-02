$PROJECT = $args[0]
$SOURCE_DIR = $args[1]
$BUILD_DIR = $args[2]

$CURR_DIR = (Get-Location)

if(! $USE_CACHE) {
  Write-Host "---- Not uploading cache. Caching disabled."
  return
}

Set-Location $SOURCE_DIR
if($PROJECT -eq "opensim-core" -and $env:APPVEYOR_REPO_BRANCH -ne "master") {
  Write-Host "---- Not uploading cache. This is not master build."
  Set-Location $CURR_DIR
  return
}

#Set-Location $env:OPENSIM_SOURCE_DIR
#git fetch origin master:master
#git checkout (git merge-base master build_cache_dev_branch)
#Set-Location $env:OPENSIM_BUILD_DIR
#cmake --build . --config Release -- /maxcpucount:4 /verbosity:quiet

$MASTERTIP = (git log -n1 --format="%H")
if($env:CMAKE_GENERATOR -like "*Win64") {
  $COMPILER = "msvc_win64"  
} else {
  $COMPILER = "msvc_win32"
}
$PACKAGE_NAME = $env:Platform + "_" + $COMPILER + "_" + "Release"
$URL="https://dl.bintray.com/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}"
$BUILD_DIRNAME = (get-item $BUILD_DIR).name
$ZIP = $BUILD_DIRNAME + ".zip"
# If already cached, return.
try {
  if ( (Invoke-WebRequest -Method Head "${URL}/${ZIP}_a").StatusCode -eq 200 ) {
    Write-Host "---- Already cached: ${PROJECT}."
    return
  }
} catch {}

Set-Location $BUILD_DIR/..
Write-Host '---- Compressing build directory into a zip.'
Add-Type -AssemblyName "System.IO.Compression.FileSystem"
[System.IO.Compression.ZipFile]::CreateFromDirectory($BUILD_DIR, "$BUILD_DIR/../$ZIP")

Write-Host '---- Splitting zip into smaller pieces for upload.'
$FILESTREAM = [System.IO.File]::OpenRead((Get-Item $ZIP))
$FILESTREAM
$BUFFER = New-Object byte[] 200mb
$LETTERS = 'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'
$LETTERS.ForEach({
  $BYTESREAD = $FILESTREAM.Read($BUFFER, 0, $BUFFER.Length)
  Write-Host $_ + $BYTESREAD
  if($BYTESREAD -eq 0) {
    Write-Host "break"
    break
  } else {
    Write-Host "write"
    $PIECE = [System.IO.File]::OpenWrite((Get-Item $ZIP).FullName + "_$_")
    $PIECE.Write($BUFFER, 0, $BYTESREAD)
    $PIECE.Close()
  }
})
Remove-Item (Get-Item $ZIP)

$PASSWORD = ConvertTo-SecureString "440061321dba00a68210b482261154ea58d03f00" -AsPlainText -Force
$CREDS = New-Object System.Management.Automation.PSCredential("klshrinidhi", $PASSWORD)
$URL = "https://api.bintray.com/content/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}/${PACKAGENAME}/${MASTERTIP}"
(Get-Item "${ZIP}_*").ForEach({
  Write-Host "---- Uploading piece $_ to opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}"
  Invoke-WebRequest -Credential $CREDS -Method PUT -InFile $_ $URL/$_ | Out-Null
})

$URL = "https://api.bintray.com/content/opensim/${PROJECT}/${PACKAGENAME}/${MASTERTIP}/publish"
Write-Host '---- Publishing uploaded build directory.'
Invoke-WebRequest -Credential $CREDS -Method POST $URL | Out-Null
Write-Host '---- Cleaning up.'
Remove-Item ${ZIP}*
Set-Location $CURR_DIR
