$PROJECT = $args[0]
$SOURCE_DIR = $args[1]

$CURR_DIR = (Get-Location)

if(! $USE_CACHE) {
  Write-Host "---- Caching disabled."
  return
}

if($env:APPVEYOR_REPO_BRANCH -ne "master") {
  Write-Host "---- Skipping cache cleanup. This is not master build."
  # return
}

Write-Host "---- Retrieving list of versions on cache."
if($env:CMAKE_GENERATOR -like "*Win64") {
  $COMPILER = "msvc_win64"  
} else {
  $COMPILER = "msvc_win32"
}
$PACKAGENAME = $env:Platform + "_" + $COMPILER + "_" + "Release"
$URL = "https://api.bintray.com/packages/opensim/${PROJECT}/${PACKAGENAME}"
choco install --yes jq > $null
$PASSWORD = ConvertTo-SecureString "440061321dba00a68210b482261154ea58d03f00" -AsPlainText -Force
$CREDS = New-Object System.Management.Automation.PSCredential("klshrinidhi", $PASSWORD)
$CACHED_VERSIONS = ((Invoke-WebRequest -Credential $CREDS $URL).Content | jq --raw-output .versions[])
[System.Collections.ArrayList]$CACHED_VERSIONS = ($CACHED_VERSIONS.split())

Write-Host "---- Retrieving list of currently used versions."
Set-Location $SOURCE_DIR
git fetch --quiet origin master:master
# Following line includes master. This is to make sure we keep cache for latest commit
# on master.
$BRANCHES = (git ls-remote --heads origin | Select-String -Pattern '.*refs/heads/(.*)')
ForEach($BRANCH in $BRANCHES) {
  $BRANCHNAME = $BRANCH.Matches.Groups[1].Value
  git fetch --quiet origin ${BRANCHNAME}:${BRANCHNAME}
  $BRANCHBASE = (git merge-base master $BRANCHNAME)
  $CACHED_VERSIONS.Remove("$BRANCHBASE")
}

$URL = "$URL/versions"
ForEach($VERSION in $CACHED_VERSIONS) {
  Write-Host "---- Deleting cache for version $VERSION."
  Invoke-WebRequest -Credential $CREDS -Method DELETE $URL/$VERSION | Out-Null
}

Set-Location $CURR_DIR
