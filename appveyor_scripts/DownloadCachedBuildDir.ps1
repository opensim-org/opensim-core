$CURR_DIR = (pwd)

New-Item -Path . -Name build -ItemType Directory -Force
if(! $USE_CACHE) {
  Write-Host "---- Not downloading cache. Caching disabled."
  return
}

Write-Host $APPVEYOR_PULL_REQUEST_NUMBER
Write-Host $APPVEYOR_REPO_BRANCH
Write-Host $APPVEYOR_REPO_COMMIT
Write-Host $APPVEYOR_REPO_COMMIT_MESSAGE
Write-Host $Platform
Write-Host $Configuration
