#!/bin/bash

start_time=$(date +%s)

cd "$(dirname "$0")" || exit

EXT_DIR="src/multiverse_parser/external"

if [ ! -d "$EXT_DIR" ]; then
    BLENDER_DIR="$EXT_DIR"/blender/linux
    mkdir -p "$BLENDER_DIR"
    cp -r ext/blender/* "$BLENDER_DIR"/

    USD_DIR="$EXT_DIR"/USD/linux
    mkdir -p "$USD_DIR"
    cp -r USD/linux "$USD_DIR"
fi
# python -m build

end_time=$(date +%s)
elapsed=$(( end_time - start_time ))
echo "Build completed in $elapsed seconds"