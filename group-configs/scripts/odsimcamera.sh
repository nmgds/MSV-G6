#! /bin/sh
xhost + && docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/X.11-unix -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/od4/bin/odsimcamera --freq=10 --cid=111
