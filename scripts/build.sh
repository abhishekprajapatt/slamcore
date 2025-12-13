#!/bin/bash

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "${SCRIPT_DIR}" && pwd)

echo "Building slamcore in: $WORKSPACE_DIR"

if [ ! -f "$WORKSPACE_DIR/package.xml" ]; then
    echo "Error: package.xml not found. Are you in the project root?"
    exit 1
fi

mkdir -p "$WORKSPACE_DIR/build"
cd "$WORKSPACE_DIR"

echo "Running colcon build..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo ""
    echo "To source the workspace, run:"
    echo "  source install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi
