#!/bin/sh
while getopts s:f: option
    do
    case "${option}"
    in
    s) IDENTITY=${OPTARG};;
    f) MOCO_ZIP=${OPTARG};;
    esac
done

function sign_code() {
    codesign \
        --sign $IDENTITY \
        --options runtime \
        --timestamp \
        --deep \
        --force "$@"
}

function cleanup() {
    rm -rf "$MOCO_TEMP_DIR"
    echo "Deleted temporary directory $MOCO_TEMP_DIR"
}

# https://stackoverflow.com/questions/4632028/how-to-create-a-temporary-directory
MOCO_TEMP_DIR=`mktemp -d -t 'opensim-moco-unzipping'`

if [[ ! "$MOCO_TEMP_DIR" || ! -d "$MOCO_TEMP_DIR" ]]; then
    echo "Could not create temporary directory."
    exit 1
fi
echo "Created temporary directory $MOCO_TEMP_DIR."

MOCO_NAME=$(basename -s .zip $MOCO_ZIP)
MOCO_ROOT="$MOCO_TEMP_DIR/$MOCO_NAME"

unzip -d "$MOCO_TEMP_DIR" "$MOCO_ZIP"

for i in `find "$MOCO_ROOT" -name "*.dylib" -type f`; do
    sign_code "$i"
done
sign_code "$MOCO_ROOT/bin/opensim-cmd"
sign_code "$MOCO_ROOT/bin/opensim-moco"
sign_code "$MOCO_ROOT/sdk/Simbody/libexec/simbody/simbody-visualizer.app"

zip --symlinks --recurse-paths "$MOCO_TEMP_DIR/" "${MOCO_ZIP}-signed"

trap cleanup EXIT
