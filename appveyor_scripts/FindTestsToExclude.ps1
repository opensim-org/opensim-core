Write-Host '---- Finding tests to exclude.'

$TESTS = (ctest --show-only | Select-String -Pattern 'Test +#[0-9]+: (.*)')
$COUNTER = 0
ForEach($TEST in $TESTS) {
  $TESTEXE = (Get-ChildItem -Recurse -Include "${TEST}.exe")
  Write-Host $TEST --- $TESTEXE.LastWriteTime
  if($TESTEXE -and 
     $TESTEXE.LastWriteTime -lt $BUILD_START_TIMESTAMP) {
    $env:OPENSIM_EXCLUDE_TESTS = "${env:OPENSIM_EXCLUDE_TESTS}|$TEST"
    Write-Host "---- Excluding test $TEST"
    $COUNTER = $COUNTER + 1
  }
}
if($COUNTER -eq 0) {
  Write-Host "---- No tests found to exclude."
}
