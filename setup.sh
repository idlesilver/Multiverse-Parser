#!/bin/bash

start_time=$(date +%s)

cd "$(dirname "$0")" || exit

# Default value
USD_ENABLED=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --usd)
            USD_ENABLED=true
            shift
        ;;
        *)
            echo "Unknown option: $1"
            exit 1
        ;;
    esac
done

EXT_DIR=$PWD/ext

BLENDER_DIR=$EXT_DIR/blender
if [ ! -d "$BLENDER_DIR" ]; then
    mkdir -p "$BLENDER_DIR"
    BLENDER_TAR_FILE=blender-4.4.1-linux-x64.tar.xz
    curl -L -o "$EXT_DIR"/$BLENDER_TAR_FILE https://download.blender.org/release/Blender4.4/$BLENDER_TAR_FILE
    tar xf "$EXT_DIR"/$BLENDER_TAR_FILE -C "$BLENDER_DIR" --strip-components=1
    rm -f "$EXT_DIR"/$BLENDER_TAR_FILE
    
    (
        cd "$BLENDER_DIR"/4.4/python/bin || exit
        ./python3.11 -m pip install --upgrade pip build --no-warn-script-location
        ./python3.11 -m pip install bpy Pillow --no-warn-script-location
    )
fi

if [ "$USD_ENABLED" = true ]; then
    CMAKE_DIR=$EXT_DIR/CMake
    if [ ! -d "$CMAKE_DIR" ]; then
        mkdir -p "$CMAKE_DIR"
        CMAKE_TAR_FILE=cmake-4.0.1-linux-x86_64.tar.gz
        curl -L -o "$EXT_DIR"/$CMAKE_TAR_FILE https://github.com/Kitware/CMake/releases/download/v4.0.1/$CMAKE_TAR_FILE
        tar xf "$EXT_DIR"/$CMAKE_TAR_FILE -C "$CMAKE_DIR" --strip-components=1
        rm -f "$EXT_DIR"/$CMAKE_TAR_FILE
    fi
    
    USD_SRC_DIR=$EXT_DIR/OpenUSD
    if [ ! -d "$USD_SRC_DIR" ]; then
        git clone https://github.com/PixarAnimationStudios/OpenUSD.git --depth 1 --branch v25.02 "$USD_SRC_DIR"
    fi
    
    BUILD_DIR=$PWD/build
    USD_BUILD_DIR=$BUILD_DIR/USD
    if [ ! -d "$USD_BUILD_DIR" ]; then
        PYTHON_EXECUTABLE=python3
        mkdir -p "$USD_BUILD_DIR"
        PATH=$CMAKE_DIR/bin:$PATH $PYTHON_EXECUTABLE "$USD_SRC_DIR"/build_scripts/build_usd.py "$USD_BUILD_DIR" \
        --no-tests \
        --no-examples \
        --no-tutorials \
        --no-tools \
        --no-docs \
        --no-python-docs \
        --python \
        --prefer-speed-over-safety \
        --no-debug-python \
        --no-imaging \
        --no-ptex \
        --no-openvdb \
        --no-usdview \
        --no-embree \
        --no-openimageio \
        --no-opencolorio \
        --no-alembic \
        --no-hdf5 \
        --no-draco \
        --no-materialx \
        --no-onetbb \
        --no-usdValidation \
        --no-mayapy-tests \
        --no-animx-tests
        
        PATH=$CMAKE_DIR/bin:$PATH PYTHONPATH=$USD_BUILD_DIR/lib/python USD_SRC_DIR=$USD_SRC_DIR USD_BUILD_DIR=$USD_BUILD_DIR PYTHON_EXECUTABLE=$PYTHON_EXECUTABLE ./install.sh
        
        USD_BIN_DIR=$PWD/USD/linux
        if [ ! -d "$USD_BIN_DIR" ]; then
            mkdir -p "$USD_BIN_DIR/lib/"
            mkdir -p "$USD_BIN_DIR/plugin/"
        fi
        cp -rf "$USD_BUILD_DIR"/lib/* "$USD_BIN_DIR"/lib/
        cp -rf "$USD_BUILD_DIR"/plugin/* "$USD_BIN_DIR"/plugin/
    fi
fi

end_time=$(date +%s)
elapsed=$(( end_time - start_time ))

echo "Build completed in $elapsed seconds"