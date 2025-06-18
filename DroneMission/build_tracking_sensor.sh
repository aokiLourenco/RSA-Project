#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

docker build -t fleetman/tracking -f $SCRIPT_DIR/tracking.dockerfile $SCRIPT_DIR