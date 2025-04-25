#!/usr/bin/env sh

# Check if USD_SRC_DIR or USD_BUILD_DIR exists
if [ -z "$USD_SRC_DIR" ] || [ -z "$USD_BUILD_DIR" ]; then
    echo "USD_SRC_DIR or USD_BUILD_DIR is unset." >&2
    exit 1
else
    echo "USD_SRC_DIR is set to: $USD_SRC_DIR"
    echo "USD_BUILD_DIR is set to: $USD_BUILD_DIR"
    
    cp -r USD/plugin "$USD_SRC_DIR"
    
    # Loop through each folder in the directory
    for USD_PLUGIN in "$USD_SRC_DIR"/plugin/*; do
        if [ -f "$USD_PLUGIN/schema.usda" ]; then
            # Execute your command within each folder
            (cd "$USD_PLUGIN" && "$USD_BUILD_DIR"/bin/usdGenSchema schema.usda)
        fi
    done
    
    # Specify the file path
    USD_CMAKE_PATH="$USD_SRC_DIR/CMakeLists.txt"
    
    # Specify the line to add
    LINE_TO_ADD="add_subdirectory(plugin)"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$LINE_TO_ADD" "$USD_CMAKE_PATH"; then
        # Add the line to the file
        printf '\n%s' "$LINE_TO_ADD" >> "$USD_CMAKE_PATH"
    fi
    
    $PYTHON_EXECUTABLE "$USD_SRC_DIR"/build_scripts/build_usd.py "$USD_BUILD_DIR" \
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
fi