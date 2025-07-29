#!/bin/bash

start_time=$(date +%s)

cd "$(dirname "$0")" || exit

./setup.sh
tar -czf src/multiverse_parser/external.tar.gz ext/blender USD/linux USD/windows
python -m build

end_time=$(date +%s)
elapsed=$(( end_time - start_time ))
echo "Build completed in $elapsed seconds"