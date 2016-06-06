Write-Host '---- Finding tests to exclude.'

$TESTS = (ctest --show-only | Select-String -Pattern 'Test +#[0-9]+: (.*)')
$COUNTER = 0
ForEach($TEST in $TESTS) {
  $TESTNAME = $TEST.Matches.Groups[1].Value
  $TESTEXE = (Get-ChildItem -Recurse -Include "${TESTNAME}.exe")
  if($TESTEXE -and 
     $TESTEXE.LastWriteTime -lt $BUILD_START_TIMESTAMP) {
    $env:OPENSIM_EXCLUDE_TESTS = "${env:OPENSIM_TESTS_TO_EXCLUDE}|$TESTNAME"
    Write-Host "---- Excluding test $TESTNAME"
    $COUNTER = $COUNTER + 1
  }
}
if($COUNTER -eq 0) {
  Write-Host "---- No tests found to exclude."
}
