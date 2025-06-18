#!/bin/bash
xhost +local:docker

docker run \
    --rm \
    --network host \
    --name tracking \
    --privileged \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml \
    --volume="fastdds.xml:/config/fastdds.xml" \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    -p 5000:5000 \
    -p 8888:8888 \
    -it fleetman/tracking