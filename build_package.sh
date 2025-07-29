#!/bin/bash

start_time=$(date +%s)

cd "$(dirname "$0")" || exit

if [ ! -d "src/multiverse_parser/external" ]; then
    BLENDER_DIR=$PWD/src/multiverse_parser/external/blender/linux
    mkdir -p "$BLENDER_DIR"
    cp -r ext/blender/* "$BLENDER_DIR"/

    USD_DIR=$PWD/src/multiverse_parser/external/USD
    mkdir -p "$USD_DIR"
    cp -r USD/linux "$USD_DIR"/linux
fi
python -m build

end_time=$(date +%s)
elapsed=$(( end_time - start_time ))
echo "Build completed in $elapsed seconds"