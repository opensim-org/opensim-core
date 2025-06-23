#!/bin/bash
# Ensure that there are no tabs in source code.
# GREP returns 0 (true) if there are any matches, and
# we don't want any matches. If there are matches,
# print a helpful message, and make the test fail by using "false".
# The GREP command here checks for any tab characters in the the files
# that match the specified pattern. GREP does not pick up explicit tabs
# (e.g., literally a \t in a source file).


OPEN_SIM_BASE="."

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  echo "Usage: $0 [<opensim-base-dir>]

Checks source files for tab characters.

Arguments:
  <opensim-base-dir>   Optional. Path to the OpenSim base directory. Default: current directory

Examples:
  $0
  $0 ./src/OpenSim
"
  exit 0
fi
# Use the first argument if provided
if [ $# -ge 1 ]; then
  OPEN_SIM_BASE="$1"
fi

OPEN_SIM_BASE=$(realpath "$OPEN_SIM_BASE") || {
  echo "Error: '$1' is not a valid directory."
  exit 1
}

if [ ! -d "$OPEN_SIM_BASE" ]; then
  echo "Directory $OPEN_SIM_BASE does not exist."
  exit 1
fi

# Search for tab characters in source files
if grep --line-number --recursive \
        --exclude-dir="*dependencies*" \
        --exclude-dir="*snopt*" \
        --include={CMakeLists.txt,*.cpp,*.c,*.h} \
        -P "\t" "$OPEN_SIM_BASE"; then
    echo "Tabs found in the lines shown above. See CONTRIBUTING.md about tabs."
    exit 1
fi
