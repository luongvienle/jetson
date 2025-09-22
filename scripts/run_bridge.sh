#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <mapping.yaml> [--protocol udp|tcp] [--yamcs-host HOST] [--yamcs-port PORT]"
  exit 1
fi
MAPPING=$1
shift
# default args
PROTOCOL="udp"
YAMCS_HOST="127.0.0.1"
YAMCS_PORT="10015"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --protocol) PROTOCOL="$2"; shift 2 ;;
    --yamcs-host) YAMCS_HOST="$2"; shift 2 ;;
    --yamcs-port) YAMCS_PORT="$2"; shift 2 ;;
    *) shift ;;
  esac
done
echo "Sourcing ROS2 environment... (please update the path below if needed)"
# NOTE: update the next line if your ROS2 is installed in a different path
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "Warning: /opt/ros/humble/setup.bash not found. Ensure ROS2 is sourced in your shell."
fi
python3 -m bridge.bridge_node "$MAPPING" --protocol "$PROTOCOL" --yamcs-host "$YAMCS_HOST" --yamcs-port "$YAMCS_PORT"
