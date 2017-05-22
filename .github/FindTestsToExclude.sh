set -e

echo '---- Finding tests to exclude.'
MINS_SINCE_BUILD_START=$(( (($(date +%s) - $BUILD_START_TIMESTAMP) / 60) + 1 ))
ALLTESTS=$(ctest --show-only | grep '#' | sed 's/^.*: //')
COUNTER=0
for test in $ALLTESTS; do
  findres=$(find . -name "*$test" -mmin +$MINS_SINCE_BUILD_START)
  if [ "$findres" != "" ]; then
    TESTS_TO_EXCLUDE="$TESTS_TO_EXCLUDE|$test"
    echo "---- Excluding test $test."
    COUNTER=$(($COUNTER + 1))
  fi
done
if [[ $COUNTER == 0 ]]; then
  echo "---- No tests found to exclude."
fi
