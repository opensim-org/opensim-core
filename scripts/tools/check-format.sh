#!/bin/bash
# Check the changed code against the clang-format code style for the repository.
# Uses a copy of https://github.com/llvm/llvm-project/blob/main/clang/tools/clang-format/git-clang-format
# located in the scripts directory to check the formatting of the diff against main.
# The script will exit with a 1 if a diff is detected and a 0 if it isn't.
# If a diff is detected, you should run the script to fix the issues locally and commit again.

# Disable exit on error so we can handle return codes manually
set +e

# === Argument Parsing ===

FIX=0
BASE_REF="main"
OPEN_SIM_BASE="."
TOOLS_DIR="./scripts/tools"
LOG_DIR="./logs"
POSITIONAL=()

# Parse optional --fix flag
while [[ $# -gt 0 ]]; do
  case "$1" in
    -f|--fix)
      FIX=1
      shift
      ;;
    -b|--branch)
      BASE_REF="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [OPTIONS] [<opensim-base-dir>] [<tools-dir>]

Options:
  -f, --fix              Automatically apply formatting fixes if issues are found.
  -b, --branch <name>    Set the git base branch to diff against. Default: main
  -h, --help             Show this help message.

Positional Arguments:
  <opensim-base-dir>     Path to the OpenSim base directory. Default: current directory
  <tools-dir>            Path to the tools directory containing git-clang-format.py. Default: ./scripts/tools

Examples:
  $0
  $0 -b develop
  $0 --fix $OPEN_SIM_BASE $TOOLS_DIR
"
      exit 0
      ;;
    -*)
      echo "Unknown option: $1"
      exit 1
      ;;
    *)
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

# Restore positional arguments
set -- "${POSITIONAL[@]}"

# Assign positional arguments with defaults
if [ ${#POSITIONAL[@]} -gt 0 ]; then
  OPEN_SIM_BASE="${POSITIONAL[0]}"
fi

if [ ${#POSITIONAL[@]} -gt 1 ]; then
  TOOLS_DIR="${POSITIONAL[1]}"
fi

OPEN_SIM_BASE=$(realpath "$OPEN_SIM_BASE")
TOOLS_DIR=$(realpath "$TOOLS_DIR")

# Independent log directory (relative to current working directory)
LOG_FILE="$LOG_DIR/clang-format.log"
GIT_CLANG_FORMAT="$TOOLS_DIR/git-clang-format.py"

# Ensure git-clang-format.py exists
if [ ! -f "$GIT_CLANG_FORMAT" ]; then
  echo "Error: git-clang-format.py not found at $GIT_CLANG_FORMAT"
  exit 1
fi

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Clear or create the log file
: > "$LOG_FILE"

# Save current directory
ORIG_DIR=$(pwd)

# Change to OpenSim base directory
cd "$OPEN_SIM_BASE" || {
  echo "Error: Failed to cd into $OPEN_SIM_BASE"
  exit 1
}

# Run git-clang-format with logging (from OpenSim base dir)
"$GIT_CLANG_FORMAT" "$BASE_REF" --diff | tee -a "$ORIG_DIR/$LOG_FILE"
rc1=${PIPESTATUS[0]}

"$GIT_CLANG_FORMAT" "$BASE_REF" --diffstat | tee -a "$ORIG_DIR/$LOG_FILE"
rc2=${PIPESTATUS[0]}

# Combine exit codes
FORMAT_ERROR=$(( rc1 || rc2 ))

# If --fix is specified and formatting issues exist, apply the fix
if [ "$FORMAT_ERROR" -ne 0 ] && [ "$FIX" -eq 1 ]; then
  echo | tee -a "$ORIG_DIR/$LOG_FILE"
  echo "Applying automatic formatting fixes..." | tee -a "$ORIG_DIR/$LOG_FILE"
  "$GIT_CLANG_FORMAT" "$BASE_REF" | tee -a "$ORIG_DIR/$LOG_FILE"
  echo "Fixes applied. Please review and re-commit your changes." | tee -a "$ORIG_DIR/$LOG_FILE"
fi


# Return to original directory
cd "$ORIG_DIR"

# Final message
if [ "$FORMAT_ERROR" -ne 0 ]; then
  if [ "$FIX" -eq 1 ]; then
    echo "Fix mode enabled. Formatting was corrected." | tee -a "$LOG_FILE"
    FORMAT_ERROR=0
  else
    echo | tee -a "$LOG_FILE"
    echo "Formatting issues found! Run the following command to fix them:" | tee -a "$LOG_FILE"
    echo | tee -a "$LOG_FILE"
    echo "    $0 --fix --branch \"$BASE_REF\" \"$OPEN_SIM_BASE\" \"$TOOLS_DIR\"" | tee -a "$LOG_FILE"
    echo | tee -a "$LOG_FILE"
  fi
else
  echo "No formatting issues found." | tee -a "$LOG_FILE"
fi

exit $FORMAT_ERROR
