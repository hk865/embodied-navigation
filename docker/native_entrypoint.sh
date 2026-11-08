#!/usr/bin/env bash
set -e
export HOME="${HOME:-/tmp/navigation-home}"
mkdir -p "$HOME"
source /opt/ros/jazzy/setup.bash
if [[ -f /workspace/navigation/install/ros/local_setup.bash ]]; then
  source /workspace/navigation/install/ros/local_setup.bash
fi
export LD_LIBRARY_PATH="/workspace/navigation/install/dds/lib:/workspace/navigation/install/livox/lib:/workspace/navigation/install/ros/lib:${LD_LIBRARY_PATH:-}"
exec "$@"
