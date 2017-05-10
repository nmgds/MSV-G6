#! /bin/sh
docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --user=odv --group-add video --device=/dev/video1:/dev/video0 seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base /opt/opendlv.core/bin/opendlv-core-system-proxy-camera --cid=111 --freq=10
