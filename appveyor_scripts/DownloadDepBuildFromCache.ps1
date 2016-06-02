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

Write-Host $BRANCHTIP
Write-Host $SOURCEURL
