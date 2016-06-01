$CURR_DIR = (pwd)

if(! $USE_CACHE) {
  Write-Host "---- Not uploading cache. Caching disabled."
  return
}

if($env:APPVEYOR_REPO_BRANCH -neq "master") {
  Write-Host "---- Not uploading cache. This is not master build."
  return
}
