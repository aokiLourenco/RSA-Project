#!/bin/bash
docker image rm fleetman/tracking
bash build_tracking_sensor.sh
bash launch_tracking_sensor.sh