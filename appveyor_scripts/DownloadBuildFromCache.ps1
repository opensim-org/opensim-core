$CURR_DIR = (pwd)

New-Item -Path . -Name build -ItemType Directory -Force

if(! $USE_CACHE) {
  Write-Host "---- Not downloading cache. Caching disabled."
  return
}

if($env:APPVEYOR_REPO_BRANCH -eq "master") {
  Write-Host "---- Not downloading cache. This is master build."
}

Write-Host $env:APPVEYOR_PULL_REQUEST_NUMBER
Write-Host $env:APPVEYOR_REPO_BRANCH
Write-Host $env:APPVEYOR_REPO_COMMIT
Write-Host $env:APPVEYOR_REPO_COMMIT_MESSAGE
Write-Host $env:Platform
Write-Host $env:Configuration
